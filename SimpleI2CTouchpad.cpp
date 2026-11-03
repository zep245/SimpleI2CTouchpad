// SimpleI2CTouchpad - a self-contained driver for an I2C HID precision touchpad on Intel LPSS (Tiger Lake).
//
//   Intel LPSS DesignWare I2C controller (PCI)  ->  HID over I2C  ->  precision touchpad reports  ->  VoodooInput
//
// It replaces VoodooI2C + VoodooI2CHID + VoodooGPIO + VoodooI2CServices. VoodooInput (the Magic Trackpad 2
// emulation) is kept: this driver publishes "VoodooInputSupported" and sends it VoodooInputEvent messages.
//
// What it does differently:
//  * the touchpad's attention (interrupt) line is read straight from its GPIO pad register, so a report is fetched
//    within ~2 ms of the touchpad raising it, and the bus is not hammered with empty reads (no blind polling);
//  * multi-report ("hybrid") frames are assembled correctly;
//  * every finger that disappears gets an explicit lift, and a watchdog lifts fingers if the touchpad goes silent,
//    so the cursor never jumps from a stale finger and clicks are not swallowed by a finger still "down".
//
// Boot-args: -sitpoff (driver does not attach), -sitppoll (ignore the GPIO line, poll), sitpdebug=N (0 quiet, 1, 2 raw).
// Logs: log show --last boot --predicate 'eventMessage CONTAINS "SimpleI2CTouchpad"'

#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/acpi/IOACPIPlatformDevice.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <libkern/OSByteOrder.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>

#define LOG(fmt, ...) IOLog("SimpleI2CTouchpad: " fmt "\n", ##__VA_ARGS__)

// ------------------------------------------------------------------------------------------------------------------
// VoodooInput interface (same layout as VoodooInput 1.1.x VoodooInputMultitouch/*.h)
// ------------------------------------------------------------------------------------------------------------------
#define VOODOO_INPUT_IDENTIFIER          "VoodooInput Instance"
#define VOODOO_INPUT_TRANSFORM_KEY       "IOFBTransform"
#define VOODOO_INPUT_LOGICAL_MAX_X_KEY   "Logical Max X"
#define VOODOO_INPUT_LOGICAL_MAX_Y_KEY   "Logical Max Y"
#define VOODOO_INPUT_PHYSICAL_MAX_X_KEY  "Physical Max X"
#define VOODOO_INPUT_PHYSICAL_MAX_Y_KEY  "Physical Max Y"
#define VOODOO_INPUT_MAX_TRANSDUCERS     10
#define kIOMessageVoodooInputMessage     12345

enum MT2FingerType {
    kMT2FingerTypeUndefined = 0,
    kMT2FingerTypeThumb,
    kMT2FingerTypeIndexFinger,
    kMT2FingerTypeMiddleFinger,
    kMT2FingerTypeRingFinger,
    kMT2FingerTypeLittleFinger,
    kMT2FingerTypePalm,
    kMT2FingerTypeCount = kMT2FingerTypePalm
};
enum VoodooInputTransducerType { STYLUS, FINGER };
struct TouchCoordinates { UInt32 x; UInt32 y; UInt8 pressure; UInt8 width; };
struct VoodooInputTransducer {
    AbsoluteTime timestamp;
    MT2FingerType fingerType;
    UInt32 secondaryId;
    VoodooInputTransducerType type;
    bool isValid;
    bool isPhysicalButtonDown;
    bool isTransducerActive;
    bool supportsPressure;
    TouchCoordinates currentCoordinates;
    TouchCoordinates previousCoordinates;
    UInt32 maxPressure;
};
struct VoodooInputEvent {
    UInt8 contact_count;
    AbsoluteTime timestamp;
    VoodooInputTransducer transducers[VOODOO_INPUT_MAX_TRANSDUCERS];
};

// ------------------------------------------------------------------------------------------------------------------
// Synopsys DesignWare I2C registers (Linux drivers/i2c/busses/i2c-designware-core.h)
// ------------------------------------------------------------------------------------------------------------------
enum {
    DW_IC_CON = 0x00, DW_IC_TAR = 0x04, DW_IC_DATA_CMD = 0x10,
    DW_IC_SS_SCL_HCNT = 0x14, DW_IC_SS_SCL_LCNT = 0x18, DW_IC_FS_SCL_HCNT = 0x1c, DW_IC_FS_SCL_LCNT = 0x20,
    DW_IC_INTR_STAT = 0x2c, DW_IC_INTR_MASK = 0x30, DW_IC_RAW_INTR_STAT = 0x34, DW_IC_RX_TL = 0x38, DW_IC_TX_TL = 0x3c,
    DW_IC_CLR_INTR = 0x40, DW_IC_CLR_TX_ABRT = 0x54, DW_IC_CLR_STOP_DET = 0x60,
    DW_IC_ENABLE = 0x6c, DW_IC_STATUS = 0x70, DW_IC_TXFLR = 0x74, DW_IC_RXFLR = 0x78, DW_IC_SDA_HOLD = 0x7c,
    DW_IC_TX_ABRT_SOURCE = 0x80, DW_IC_ENABLE_STATUS = 0x9c,
    DW_IC_COMP_PARAM_1 = 0xf4, DW_IC_COMP_VERSION = 0xf8, DW_IC_COMP_TYPE = 0xfc,
};
#define DW_IC_COMP_TYPE_VALUE    0x44570140u
#define DW_IC_SDA_HOLD_MIN_VERS  0x3131312Au
#define DW_IC_CON_MASTER         0x01
#define DW_IC_CON_SPEED_STD      0x02
#define DW_IC_CON_SPEED_FAST     0x04
#define DW_IC_CON_RESTART_EN     0x20
#define DW_IC_CON_SLAVE_DISABLE  0x40
#define DW_IC_INTR_TX_ABRT       (1u << 6)
#define DW_IC_INTR_STOP_DET      (1u << 9)
#define DW_IC_DATA_CMD_READ      (1u << 8)
#define DW_IC_DATA_CMD_STOP      (1u << 9)
#define DW_IC_DATA_CMD_RESTART   (1u << 10)
#define LPSS_PRIV_RESETS         0x204   // LPSS private space at 0x200, resets register at +4
#define LPSS_PRIV_RESETS_VALUE   0x7     // FUNC | IDMA out of reset

// HID over I2C (Microsoft spec 1.0)
enum { HID_RESET = 1, HID_GET_REPORT = 2, HID_SET_REPORT = 3, HID_SET_POWER = 8 };
enum { HID_REPORT_INPUT = 1, HID_REPORT_FEATURE = 3 };

// ------------------------------------------------------------------------------------------------------------------
// HID report descriptor model
// ------------------------------------------------------------------------------------------------------------------
struct HidField {
    UInt8  rid;
    UInt8  kind;        // 0 input, 2 feature
    UInt8  bits;
    SInt8  uexp;
    UInt16 page, usage;
    UInt16 appPage, appUsage;
    SInt16 finger;      // serial number of the enclosing Finger collection, -1 if none
    UInt32 bitOff;
    UInt32 unit;
    SInt32 lmin, lmax, pmin, pmax;
};
#define MAX_FIELDS   768
#define MAX_FINGERS  10

struct FieldRef { bool ok; HidField f; };
struct FingerFields { FieldRef tip, conf, id, x, y, pressure, width, height; };
struct PtpLayout {
    bool   valid;
    bool   usesIds;
    UInt8  rid;               // input report with the fingers
    UInt32 inBytes;           // payload bytes of that report (without the id)
    int    nFingers;
    FingerFields fingers[MAX_FINGERS];
    FieldRef contactCount, scanTime, button;
    FieldRef inputMode, surfaceSwitch, buttonSwitch, contactMax, padType, latency;
};

static UInt32 getBitsU(const UInt8 *d, UInt32 dlen, UInt32 off, UInt8 bits) {
    UInt32 v = 0;
    for (UInt32 b = 0; b < bits && b < 32; b++) {
        UInt32 bit = off + b;
        if (bit / 8 >= dlen) break;
        if ((d[bit / 8] >> (bit % 8)) & 1) v |= (1u << b);
    }
    return v;
}
static SInt32 getField(const UInt8 *d, UInt32 dlen, const HidField &f) {
    UInt32 v = getBitsU(d, dlen, f.bitOff, f.bits);
    if (f.lmin < 0 && f.bits > 0 && f.bits < 32 && (v & (1u << (f.bits - 1)))) v |= ~((1u << f.bits) - 1);
    return (SInt32)v;
}
static void setField(UInt8 *d, UInt32 dlen, const HidField &f, UInt32 val) {
    for (UInt32 b = 0; b < f.bits && b < 32; b++) {
        UInt32 bit = f.bitOff + b;
        if (bit / 8 >= dlen) break;
        if ((val >> b) & 1) d[bit / 8] |= (UInt8)(1u << (bit % 8));
        else d[bit / 8] &= (UInt8)~(1u << (bit % 8));
    }
}

// Physical size of an axis in 0.01 mm (the unit VoodooInput expects).
static UInt32 physicalMax001mm(const HidField &f) {
    SInt64 v = (SInt64)f.pmax - (SInt64)f.pmin;
    if (v <= 0) return 0;
    int e = f.uexp;
    UInt32 sys = f.unit & 0xF, len = (f.unit >> 4) & 0xF;
    if (sys == 1 && len == 1) e += 3;                 // centimetre -> 0.01 mm
    else if (sys == 3 && len == 1) { v *= 254; e += 1; } // inch -> 0.01 mm
    else e += 3;                                      // unknown unit: assume cm
    while (e > 0) { v *= 10; e--; }
    while (e < 0) { v /= 10; e++; }
    if (v > 0x7fffffff) v = 0x7fffffff;
    return (UInt32)v;
}

class HidParser {
public:
    HidField *fields = nullptr;
    int nFields = 0;
    bool usesIds = false;
    UInt32 inBits[256], featBits[256];

    bool parse(const UInt8 *d, UInt32 len) {
        struct Glob { UInt16 page; SInt32 lmin, lmax, pmin, pmax; SInt8 uexp; UInt32 unit, size, count; UInt8 rid; };
        Glob g = {}, stack[8]; int sp = 0;
        UInt32 usages[64]; bool usageHasPage[64]; int nUsages = 0; UInt32 umin = 0, umax = 0; bool hasMin = false, hasMax = false;
        struct Coll { UInt16 page, usage; UInt8 type; SInt16 finger; };
        Coll cs[16]; int depth = 0;
        UInt16 appPage = 0, appUsage = 0; SInt16 fingerSerial = 0;
        memset(inBits, 0, sizeof(inBits)); memset(featBits, 0, sizeof(featBits));
        nFields = 0; usesIds = false;
        UInt32 i = 0;
        while (i < len) {
            UInt8 pfx = d[i];
            if (pfx == 0xFE) { // long item
                if (i + 2 >= len) break;
                i += 3 + d[i + 1];
                continue;
            }
            UInt32 sz = pfx & 3; if (sz == 3) sz = 4;
            UInt32 type = (pfx >> 2) & 3, tag = pfx >> 4;
            if (i + 1 + sz > len) return false;
            UInt32 u = 0;
            for (UInt32 k = 0; k < sz; k++) u |= (UInt32)d[i + 1 + k] << (8 * k);
            SInt32 s = (SInt32)u;
            if (sz == 1) s = (SInt8)u; else if (sz == 2) s = (SInt16)u;
            i += 1 + sz;
            if (type == 1) { // global
                switch (tag) {
                    case 0: g.page = (UInt16)u; break;
                    case 1: g.lmin = s; break;
                    case 2: g.lmax = (g.lmin < 0) ? s : (SInt32)u; break;
                    case 3: g.pmin = s; break;
                    case 4: g.pmax = (g.pmin < 0) ? s : (SInt32)u; break;
                    case 5: g.uexp = (SInt8)((u & 0x8) ? (SInt32)(u | ~0xFu) : (SInt32)(u & 0xF)); break;
                    case 6: g.unit = u; break;
                    case 7: g.size = u; break;
                    case 8: g.rid = (UInt8)u; usesIds = true; break;
                    case 9: g.count = u; break;
                    case 10: if (sp < 8) stack[sp++] = g; break;
                    case 11: if (sp > 0) g = stack[--sp]; break;
                }
            } else if (type == 2) { // local
                switch (tag) {
                    case 0: if (nUsages < 64) { usageHasPage[nUsages] = (sz == 4); usages[nUsages++] = u; } break;
                    case 1: umin = u; hasMin = true; break;
                    case 2: umax = u; hasMax = true; break;
                }
            } else if (type == 0) { // main
                if (tag == 8 || tag == 0xB) { // Input / Feature
                    UInt32 *bitsArr = (tag == 8) ? inBits : featBits;
                    UInt32 off = bitsArr[g.rid];
                    bool constant = u & 1, variable = u & 2;
                    if (!constant && variable) {
                        for (UInt32 c = 0; c < g.count && c < 128; c++) {
                            UInt32 us; bool hp = false;
                            if (c < (UInt32)nUsages) { us = usages[c]; hp = usageHasPage[c]; }
                            else if (nUsages > 0) { us = usages[nUsages - 1]; hp = usageHasPage[nUsages - 1]; }
                            else if (hasMin && hasMax) us = (umin + c > umax) ? umax : umin + c;
                            else us = 0;
                            if (nFields < MAX_FIELDS && g.size <= 32) {
                                HidField &f = fields[nFields++];
                                f.rid = g.rid; f.kind = (tag == 8) ? 0 : 2; f.bits = (UInt8)g.size; f.uexp = g.uexp;
                                f.page = hp ? (UInt16)(us >> 16) : g.page;
                                f.usage = (UInt16)(us & 0xFFFF);
                                f.appPage = appPage; f.appUsage = appUsage;
                                f.finger = -1;
                                for (int k = depth - 1; k >= 0; k--) if (cs[k].finger >= 0) { f.finger = cs[k].finger; break; }
                                f.bitOff = off + c * g.size; f.unit = g.unit;
                                f.lmin = g.lmin; f.lmax = g.lmax; f.pmin = g.pmin; f.pmax = g.pmax;
                                if (f.pmin == 0 && f.pmax == 0) { f.pmin = f.lmin; f.pmax = f.lmax; }
                            }
                        }
                    }
                    bitsArr[g.rid] = off + g.size * g.count;
                } else if (tag == 0xA) { // Collection
                    UInt32 us = nUsages ? usages[0] : 0;
                    UInt16 page = (nUsages && usageHasPage[0]) ? (UInt16)(us >> 16) : g.page, usage = (UInt16)(us & 0xFFFF);
                    if (depth < 16) {
                        cs[depth].page = page; cs[depth].usage = usage; cs[depth].type = (UInt8)u;
                        cs[depth].finger = (page == 0x0D && usage == 0x22) ? fingerSerial++ : -1;
                        if (depth == 0) { appPage = page; appUsage = usage; }
                    }
                    depth++;
                } else if (tag == 0xC) { // End Collection
                    if (depth > 0) depth--;
                }
                nUsages = 0; hasMin = hasMax = false;
            }
        }
        return true;
    }

    // Build the precision-touchpad layout. Returns false if no touchpad report was found.
    bool layout(PtpLayout &L) {
        memset(&L, 0, sizeof(L));
        L.usesIds = usesIds;
        // the input report that carries Contact Count, preferably inside the Touch Pad application collection
        int best = -1;
        for (int k = 0; k < nFields; k++) {
            const HidField &f = fields[k];
            if (f.kind == 0 && f.page == 0x0D && f.usage == 0x54) {
                if (best < 0 || (f.appPage == 0x0D && f.appUsage == 0x05)) best = k;
            }
        }
        if (best < 0) return false;
        L.rid = fields[best].rid;
        L.inBytes = (inBits[L.rid] + 7) / 8;
        SInt16 serials[MAX_FINGERS]; int ns = 0;
        for (int k = 0; k < nFields; k++) {
            const HidField &f = fields[k];
            if (f.kind == 2) {
                FieldRef *r = nullptr;
                if (f.page == 0x0D && f.usage == 0x52) r = &L.inputMode;
                else if (f.page == 0x0D && f.usage == 0x57) r = &L.surfaceSwitch;
                else if (f.page == 0x0D && f.usage == 0x58) r = &L.buttonSwitch;
                else if (f.page == 0x0D && f.usage == 0x55) r = &L.contactMax;
                else if (f.page == 0x0D && f.usage == 0x59) r = &L.padType;
                else if (f.page == 0x0D && f.usage == 0x60) r = &L.latency;
                if (r && !r->ok) { r->ok = true; r->f = f; }
                continue;
            }
            if (f.rid != L.rid) continue;
            if (f.finger < 0) {
                if (f.page == 0x0D && f.usage == 0x54 && !L.contactCount.ok) { L.contactCount.ok = true; L.contactCount.f = f; }
                else if (f.page == 0x0D && f.usage == 0x56 && !L.scanTime.ok) { L.scanTime.ok = true; L.scanTime.f = f; }
                else if (f.page == 0x09 && f.usage >= 1 && f.usage <= 3 && !L.button.ok) { L.button.ok = true; L.button.f = f; }
                continue;
            }
            int slot = -1;
            for (int s = 0; s < ns; s++) if (serials[s] == f.finger) { slot = s; break; }
            if (slot < 0) {
                if (ns >= MAX_FINGERS) continue;
                serials[ns] = f.finger; slot = ns++;
            }
            FingerFields &F = L.fingers[slot];
            FieldRef *r = nullptr;
            if (f.page == 0x0D && f.usage == 0x42) r = &F.tip;
            else if (f.page == 0x0D && f.usage == 0x47) r = &F.conf;
            else if (f.page == 0x0D && f.usage == 0x51) r = &F.id;
            else if (f.page == 0x0D && f.usage == 0x30) r = &F.pressure;
            else if (f.page == 0x0D && f.usage == 0x48) r = &F.width;
            else if (f.page == 0x0D && f.usage == 0x49) r = &F.height;
            else if (f.page == 0x01 && f.usage == 0x30) r = &F.x;
            else if (f.page == 0x01 && f.usage == 0x31) r = &F.y;
            if (r && !r->ok) { r->ok = true; r->f = f; }
        }
        L.nFingers = ns;
        L.valid = ns > 0 && L.fingers[0].x.ok && L.fingers[0].y.ok && L.fingers[0].tip.ok;
        return L.valid;
    }
};

// ------------------------------------------------------------------------------------------------------------------
// The driver
// ------------------------------------------------------------------------------------------------------------------
struct Contact { UInt32 id; bool tip, conf; SInt32 x, y; };
struct SlotState { bool active; UInt32 id; UInt32 x, y; };

class SimpleI2CTouchpad : public IOService {
    OSDeclareDefaultStructors(SimpleI2CTouchpad)
public:
    IOService *probe(IOService *provider, SInt32 *score) override;
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;
    void free() override;
    IOReturn setPowerState(unsigned long state, IOService *dev) override;
    bool handleOpen(IOService *client, IOOptionBits opts, void *arg) override;
    bool handleIsOpen(const IOService *client) const override;
    void handleClose(IOService *client, IOOptionBits opts) override;

private:
    enum State { kStateInit, kStateRun, kStateAsleep, kStateDead };

    IOPCIDevice *pci = nullptr;
    IOACPIPlatformDevice *ctrlAcpi = nullptr;   // \_SB.PC00.I2Cx
    IOACPIPlatformDevice *devAcpi = nullptr;    // the touchpad (TPD0)
    IOMemoryMap *mmio = nullptr;
    volatile UInt8 *regs = nullptr;
    IOWorkLoop *wl = nullptr;
    IOCommandGate *gate = nullptr;
    IOTimerEventSource *timer = nullptr;
    IOService *voodooInput = nullptr;

    // bus
    UInt32 txDepth = 32, rxDepth = 32, ssH = 0, ssL = 0, fsH = 0, fsL = 0, sdaHold = 0, compVer = 0;
    UInt16 addr = 0x2C;
    UInt32 busHz = 400000;

    // attention line (GPIO pad)
    IOMemoryMap *gpioMap = nullptr;
    volatile UInt32 *padDW0 = nullptr;
    bool gpioOK = false;
    UInt32 apicIrq = 0xFFFFFFFF;

    // HID
    UInt16 descReg = 0x20;
    UInt8 hidDesc[30];
    UInt16 repDescLen = 0, repDescReg = 0, inputReg = 0, maxInput = 0, outputReg = 0, cmdReg = 0, dataReg = 0;
    UInt16 vid = 0, pid = 0, ver = 0;
    UInt8 *inBuf = nullptr; UInt32 inBufLen = 0;
    UInt8 *repDesc = nullptr;
    HidParser parser;
    PtpLayout L;
    UInt32 logMaxX = 0, logMaxY = 0, physMaxX = 0, physMaxY = 0;
    SInt32 minX = 0, minY = 0;

    // engine
    State state = kStateInit;
    int initFailures = 0;
    bool published = false;
    bool pmStarted = false;
    Contact frame[MAX_FINGERS * 2]; int frameN = 0, frameExpected = -1; bool frameButton = false;
    SlotState slots[16];
    bool lastSentEmpty = true, lastButton = false;
    UInt64 lastReportNs = 0, lastTouchNs = 0, lastStatsNs = 0;
    int consecutiveEmpty = 0;
    VoodooInputEvent ev;

    // debug / stats
    UInt32 debugLevel = 1;
    UInt32 watchdogMs = 300;
    UInt32 nReports = 0, nFrames = 0, nEmpty = 0, nErrors = 0, nSynthLifts = 0, nWatchdog = 0, nForeign = 0, nPartial = 0;

    // helpers
    UInt32 rd(UInt32 off) { return *(volatile UInt32 *)(regs + off); }
    void wr(UInt32 off, UInt32 v) { *(volatile UInt32 *)(regs + off) = v; }
    static UInt64 nowNs() { UInt64 t, ns; clock_get_uptime(&t); absolutetime_to_nanoseconds(t, &ns); return ns; }
    UInt32 numProp(const char *key, UInt32 def) {
        OSNumber *n = OSDynamicCast(OSNumber, getProperty(key));
        return n ? n->unsigned32BitValue() : def;
    }

    bool findAcpi();
    void readAcpiConfig();
    void setupGpio();
    int padLevel();
    void configurePci();
    bool mapController();
    bool setEnable(bool on);
    bool initBus();
    IOReturn xfer(const UInt8 *w, UInt32 wlen, UInt8 *r, UInt32 rlen);
    IOReturn hidCommand(UInt8 opcode, UInt8 rtype, UInt8 rid, const UInt8 *data, UInt32 dlen, UInt8 *rbuf, UInt32 rlen);
    bool readHidDescriptor();
    bool waitAttention(UInt32 ms);
    int readInput();  // 1 report, 0 empty, -1 error
    bool hidInit(bool first);
    void setModes();
    void publish();
    void handleReport(const UInt8 *p, UInt32 n);
    void flushFrame();
    void sendLiftAll(const char *why);
    void updateStats();
    void timerFired(IOTimerEventSource *src);
    IOReturn gatedPower(void *a0, void *a1, void *a2, void *a3);
    void hexdump(const char *what, const UInt8 *d, UInt32 n);
};

OSDefineMetaClassAndStructors(SimpleI2CTouchpad, IOService)

static IOPMPowerState gPowerStates[2] = {
    { kIOPMPowerStateVersion1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { kIOPMPowerStateVersion1, kIOPMPowerOn | kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0 },
};

void SimpleI2CTouchpad::hexdump(const char *what, const UInt8 *d, UInt32 n) {
    char line[3 * 32 + 1];
    for (UInt32 o = 0; o < n; o += 32) {
        UInt32 m = (n - o < 32) ? n - o : 32, k = 0;
        for (UInt32 j = 0; j < m; j++) {
            static const char hx[] = "0123456789abcdef";
            line[k++] = hx[d[o + j] >> 4]; line[k++] = hx[d[o + j] & 15]; line[k++] = ' ';
        }
        line[k] = 0;
        LOG("%s +%03x: %s", what, o, line);
    }
}

// ---------------------------------------------------------------- ACPI ------------------------------------------
static bool entryHasId(IORegistryEntry *e, const char *id) {
    const char *keys[] = { "compatible", "name", "IOName" };
    size_t idl = strlen(id);
    for (const char *k : keys) {
        OSObject *o = e->getProperty(k);
        if (OSString *s = OSDynamicCast(OSString, o)) { if (s->isEqualTo(id)) return true; }
        if (OSData *dd = OSDynamicCast(OSData, o)) {
            const char *p = (const char *)dd->getBytesNoCopy(); unsigned len = dd->getLength();
            for (unsigned a = 0; p && a < len;) { // NUL separated list
                size_t l = strnlen(p + a, len - a);
                if (l == idl && !strncmp(p + a, id, l)) return true;
                a += l + 1;
            }
        }
    }
    if (IOACPIPlatformDevice *a = OSDynamicCast(IOACPIPlatformDevice, e)) {
        OSObject *o = nullptr;
        if (a->evaluateObject("_CID", &o) == kIOReturnSuccess && o) {
            bool m = false;
            if (OSString *s = OSDynamicCast(OSString, o)) m = s->isEqualTo(id);
            if (OSNumber *n = OSDynamicCast(OSNumber, o)) m = !strcmp(id, "PNP0C50") && n->unsigned32BitValue() == 0x500CD041;
            o->release();
            if (m) return true;
        }
    }
    return false;
}

bool SimpleI2CTouchpad::findAcpi() {
    OSSafeReleaseNULL(ctrlAcpi); OSSafeReleaseNULL(devAcpi);
    OSString *path = OSDynamicCast(OSString, pci->getProperty("acpi-path"));
    if (!path) return false;
    IORegistryEntry *e = IORegistryEntry::fromPath(path->getCStringNoCopy());
    ctrlAcpi = OSDynamicCast(IOACPIPlatformDevice, e);
    if (!ctrlAcpi) { OSSafeReleaseNULL(e); return false; }
    OSIterator *it = ctrlAcpi->getChildIterator(gIOACPIPlane);
    if (!it) return false;
    while (IORegistryEntry *c = OSDynamicCast(IORegistryEntry, it->getNextObject())) {
        IOACPIPlatformDevice *a = OSDynamicCast(IOACPIPlatformDevice, c);
        if (!a) continue;
        UInt32 sta = 0x0F;
        if (a->evaluateInteger("_STA", &sta) != kIOReturnSuccess) sta = 0x0F;
        if (!(sta & 1)) continue;
        if (entryHasId(a, "PNP0C50")) { devAcpi = a; devAcpi->retain(); break; }
    }
    it->release();
    return devAcpi != nullptr;
}

void SimpleI2CTouchpad::readAcpiConfig() {
    // I2C address / speed and the APIC interrupt number from the touchpad's _CRS
    OSObject *o = nullptr;
    if (devAcpi->evaluateObject("_CRS", &o) == kIOReturnSuccess && o) {
        if (OSData *crs = OSDynamicCast(OSData, o)) {
            const UInt8 *d = (const UInt8 *)crs->getBytesNoCopy(); UInt32 n = crs->getLength(), i = 0;
            while (d && i < n) {
                UInt8 b = d[i];
                if (b & 0x80) {
                    if (i + 3 > n) break;
                    UInt32 l = d[i + 1] | (d[i + 2] << 8);
                    if (b == 0x8E && i + 18 <= n && d[i + 5] == 1) { // I2cSerialBus
                        busHz = d[i + 12] | (d[i + 13] << 8) | (d[i + 14] << 16) | ((UInt32)d[i + 15] << 24);
                        addr = d[i + 16] | (d[i + 17] << 8);
                    } else if (b == 0x89 && i + 9 <= n && d[i + 4] >= 1) { // Extended Interrupt
                        apicIrq = d[i + 5] | (d[i + 6] << 8) | (d[i + 7] << 16) | ((UInt32)d[i + 8] << 24);
                    }
                    i += 3 + l;
                } else {
                    if (((b >> 3) & 0xF) == 0xF) break; // end tag
                    i += 1 + (b & 7);
                }
            }
        }
        o->release();
    }
    // HID descriptor register from _DSM(3cdff6f7-4267-4555-ad05-b30a3d8938de, 1, 1)
    static const UInt8 uuid[16] = { 0xF7, 0xF6, 0xDF, 0x3C, 0x67, 0x42, 0x55, 0x45, 0xAD, 0x05, 0xB3, 0x0A, 0x3D, 0x89, 0x38, 0xDE };
    OSObject *params[4] = { OSData::withBytes(uuid, 16), OSNumber::withNumber(1ULL, 32), OSNumber::withNumber(1ULL, 32), OSArray::withCapacity(1) };
    o = nullptr;
    if (devAcpi->evaluateObject("_DSM", &o, params, 4) == kIOReturnSuccess && o) {
        if (OSNumber *nn = OSDynamicCast(OSNumber, o)) descReg = (UInt16)nn->unsigned32BitValue();
        o->release();
    } else {
        LOG("_DSM for the HID descriptor register failed - using 0x%x", descReg);
    }
    for (int k = 0; k < 4; k++) OSSafeReleaseNULL(params[k]);
    // bus timing from the controller's FMCN / SSCN
    o = nullptr;
    if (ctrlAcpi->evaluateObject("FMCN", &o) == kIOReturnSuccess && o) {
        if (OSArray *a = OSDynamicCast(OSArray, o)) {
            OSNumber *h = OSDynamicCast(OSNumber, a->getObject(0)), *l = OSDynamicCast(OSNumber, a->getObject(1)), *s = OSDynamicCast(OSNumber, a->getObject(2));
            if (h) fsH = h->unsigned32BitValue();
            if (l) fsL = l->unsigned32BitValue();
            if (s) sdaHold = s->unsigned32BitValue();
        }
        o->release();
    }
    o = nullptr;
    if (ctrlAcpi->evaluateObject("SSCN", &o) == kIOReturnSuccess && o) {
        if (OSArray *a = OSDynamicCast(OSArray, o)) {
            OSNumber *h = OSDynamicCast(OSNumber, a->getObject(0)), *l = OSDynamicCast(OSNumber, a->getObject(1));
            if (h) ssH = h->unsigned32BitValue();
            if (l) ssL = l->unsigned32BitValue();
        }
        o->release();
    }
    // defaults for a 120 MHz LPSS clock (Linux i2c-designware formulas) when ACPI gives nothing
    if (!fsH || !fsL) { fsH = 120 * (600 + 300) / 1000 - 3; fsL = 120 * (1300 + 300) / 1000 - 1; }
    if (!ssH || !ssL) { ssH = 120 * (4000 + 300) / 1000 - 3; ssL = 120 * (4700 + 300) / 1000 - 1; }
    // Info.plist overrides
    addr = (UInt16)numProp("I2CAddress", addr);
    descReg = (UInt16)numProp("HIDDescriptorRegister", descReg);
}

// The touchpad's attention line: its GPIO pad's RX state (PADCFG DW0 bit 1), read directly. Low = report waiting.
// The pad's address comes from the DSDT's own GPIO library: \_SB.GADR(group, 2) + pad * 0x10.
void SimpleI2CTouchpad::setupGpio() {
    gpioOK = false;
    UInt32 pad = numProp("GPIOPad", 0);
    int dummy[4];
    if (!pad || numProp("ForcePolling", 0) || PE_parse_boot_argn("-sitppoll", dummy, sizeof(dummy))) { LOG("attention line: polling (no GPIOPad / ForcePolling)"); return; }
    UInt32 group = (pad >> 16) & 0xFF, num = pad & 0xFFFF;
    UInt64 base = 0;
    OSObject *params[2] = { OSNumber::withNumber((UInt64)group, 32), OSNumber::withNumber(2ULL, 32) };
    OSObject *o = nullptr;
    IOReturn r = devAcpi->evaluateObject("\\_SB.GADR", &o, params, 2);
    if (r != kIOReturnSuccess || !o) {
        IORegistryEntry *sb = IORegistryEntry::fromPath("/_SB", gIOACPIPlane);
        if (IOACPIPlatformDevice *a = OSDynamicCast(IOACPIPlatformDevice, sb)) r = a->evaluateObject("GADR", &o, params, 2);
        OSSafeReleaseNULL(sb);
    }
    for (int k = 0; k < 2; k++) OSSafeReleaseNULL(params[k]);
    if (r == kIOReturnSuccess && o) {
        if (OSNumber *n = OSDynamicCast(OSNumber, o)) base = n->unsigned64BitValue();
        o->release();
    }
    if (!base) { LOG("attention line: \\_SB.GADR failed (0x%x) - polling", r); return; }
    UInt64 phys = base + (UInt64)num * 0x10;
    UInt64 page = phys & ~0xFFFULL;
    IOMemoryDescriptor *md = IOMemoryDescriptor::withAddressRange(page, 0x1000, kIODirectionInOut | kIOMemoryMapperNone, nullptr);
    if (!md) { LOG("attention line: no descriptor for 0x%llx", phys); return; }
    gpioMap = md->map(kIOMapInhibitCache);
    md->release();
    if (!gpioMap) { LOG("attention line: map of 0x%llx failed", phys); return; }
    padDW0 = (volatile UInt32 *)(gpioMap->getVirtualAddress() + (phys - page));
    UInt32 dw0 = padDW0[0], dw1 = padDW0[1];
    LOG("attention line: pad 0x%08x at 0x%llx DW0=0x%08x DW1=0x%08x (mode %u, rxdis %u, apic-route %u, int %u, level %u), _CRS irq 0x%x",
        pad, phys, dw0, dw1, (dw0 >> 10) & 7, (dw0 >> 9) & 1, (dw0 >> 20) & 1, dw1 & 0x7F, (dw0 >> 1) & 1, apicIrq);
    if (dw0 == 0xFFFFFFFF || ((dw0 >> 10) & 7) != 0 || ((dw0 >> 9) & 1)) {
        LOG("attention line: pad is not a GPIO input - polling");
        return;
    }
    if (apicIrq != 0xFFFFFFFF && (dw1 & 0x7F) != (apicIrq & 0x7F)) {
        LOG("attention line: pad interrupt %u does not match _CRS 0x%x - polling", dw1 & 0x7F, apicIrq);
        return;
    }
    gpioOK = true;
    setProperty("Attention", "GPIO pad");
}

int SimpleI2CTouchpad::padLevel() {
    if (!gpioOK || !padDW0) return -1;
    return (padDW0[0] >> 1) & 1;
}

// ---------------------------------------------------------------- controller -------------------------------------
void SimpleI2CTouchpad::configurePci() {
    IOByteCount off = 0;
    if (pci->extendedFindPCICapability(kIOPCIPowerManagementCapability, &off)) {
        UInt32 v = pci->extendedConfigRead32(off + 4);
        if (v & 3) LOG("controller was in D%u - setting D0", v & 3);
        pci->extendedConfigWrite32(off + 4, v & ~3u);
        IOSleep(10);
    }
    pci->setBusMasterEnable(true);
    pci->setMemoryEnable(true);
}

bool SimpleI2CTouchpad::mapController() {
    OSSafeReleaseNULL(mmio); regs = nullptr;
    mmio = pci->mapDeviceMemoryWithIndex(0);
    if (!mmio) return false;
    regs = (volatile UInt8 *)mmio->getVirtualAddress();
    wr(LPSS_PRIV_RESETS, LPSS_PRIV_RESETS_VALUE);
    IODelay(100);
    UInt32 type = rd(DW_IC_COMP_TYPE);
    if (type != DW_IC_COMP_TYPE_VALUE) { LOG("controller type 0x%08x is not DesignWare", type); return false; }
    return true;
}

bool SimpleI2CTouchpad::setEnable(bool on) {
    for (int k = 0; k < 200; k++) {
        wr(DW_IC_ENABLE, on ? 1 : 0);
        if ((rd(DW_IC_ENABLE_STATUS) & 1) == (on ? 1u : 0u)) return true;
        IODelay(25);
    }
    LOG("controller did not %s", on ? "enable" : "disable");
    return false;
}

bool SimpleI2CTouchpad::initBus() {
    if (!setEnable(false)) return false;
    UInt32 p = rd(DW_IC_COMP_PARAM_1);
    if (p && p != 0xFFFFFFFF) { txDepth = ((p >> 16) & 0xFF) + 1; rxDepth = ((p >> 8) & 0xFF) + 1; }
    compVer = rd(DW_IC_COMP_VERSION);
    wr(DW_IC_SS_SCL_HCNT, ssH); wr(DW_IC_SS_SCL_LCNT, ssL);
    wr(DW_IC_FS_SCL_HCNT, fsH); wr(DW_IC_FS_SCL_LCNT, fsL);
    if (compVer >= DW_IC_SDA_HOLD_MIN_VERS) {
        UInt32 h = sdaHold ? sdaHold : rd(DW_IC_SDA_HOLD);
        if (!(h & 0xFF0000)) h |= 1u << 16; // non-zero SDA RX hold (Linux workaround for arbitration loss)
        wr(DW_IC_SDA_HOLD, h);
    }
    wr(DW_IC_TX_TL, txDepth / 2); wr(DW_IC_RX_TL, 0);
    UInt32 speed = (busHz > 100000) ? DW_IC_CON_SPEED_FAST : DW_IC_CON_SPEED_STD;
    wr(DW_IC_CON, DW_IC_CON_MASTER | DW_IC_CON_SLAVE_DISABLE | DW_IC_CON_RESTART_EN | speed);
    wr(DW_IC_INTR_MASK, 0);
    return true;
}

// One I2C transaction: optional write of wlen bytes, then (repeated start) read of rlen bytes, then stop.
// Polled; while the last bytes are on the wire the thread sleeps instead of spinning.
IOReturn SimpleI2CTouchpad::xfer(const UInt8 *w, UInt32 wlen, UInt8 *r, UInt32 rlen) {
    if (!regs) return kIOReturnNotReady;
    UInt32 total = wlen + rlen;
    if (!total) return kIOReturnBadArgument;
    if (!setEnable(false)) { initBus(); return kIOReturnBusy; }
    wr(DW_IC_TAR, addr & 0x3FF);
    wr(DW_IC_INTR_MASK, 0);
    (void)rd(DW_IC_CLR_INTR);
    if (!setEnable(true)) return kIOReturnBusy;
    UInt64 deadline = nowNs() + 25000000ULL + (UInt64)total * 60000ULL;
    UInt32 sent = 0, got = 0;
    IOReturn ret = kIOReturnSuccess;
    for (;;) {
        UInt32 txfl = rd(DW_IC_TXFLR);
        while (sent < total && txfl < txDepth) {
            if (sent >= wlen && (sent - wlen) - got >= rxDepth) break; // never more reads in flight than the RX FIFO holds
            UInt32 cmd;
            if (sent < wlen) cmd = w[sent];
            else { cmd = DW_IC_DATA_CMD_READ; if (sent == wlen && wlen) cmd |= DW_IC_DATA_CMD_RESTART; }
            if (sent == total - 1) cmd |= DW_IC_DATA_CMD_STOP;
            wr(DW_IC_DATA_CMD, cmd);
            sent++; txfl++;
        }
        for (UInt32 n = rd(DW_IC_RXFLR); n && got < rlen; n--) r[got++] = (UInt8)rd(DW_IC_DATA_CMD);
        UInt32 raw = rd(DW_IC_RAW_INTR_STAT);
        if (raw & DW_IC_INTR_TX_ABRT) {
            UInt32 src = rd(DW_IC_TX_ABRT_SOURCE);
            (void)rd(DW_IC_CLR_TX_ABRT);
            if (debugLevel >= 2 || nErrors < 20) LOG("i2c abort: source 0x%x (w%u r%u, sent %u got %u)", src, wlen, rlen, sent, got);
            ret = (src & 1) ? kIOReturnNoDevice : kIOReturnIOError;
            break;
        }
        if (sent == total && got == rlen) {
            if (raw & DW_IC_INTR_STOP_DET) break;
        }
        if (nowNs() > deadline) {
            LOG("i2c timeout (w%u r%u, sent %u got %u, status 0x%x)", wlen, rlen, sent, got, rd(DW_IC_STATUS));
            ret = kIOReturnTimeout;
            break;
        }
        // All commands queued: wait for the wire. Sleep when more than ~8 bytes are left (400 kHz: ~23 us/byte).
        if (sent == total && rlen - got > 8) IOSleep(1);
        else IODelay(10);
    }
    (void)rd(DW_IC_CLR_STOP_DET);
    setEnable(false);
    if (ret != kIOReturnSuccess) { nErrors++; initBus(); }
    return ret;
}

// ---------------------------------------------------------------- HID over I2C -----------------------------------
IOReturn SimpleI2CTouchpad::hidCommand(UInt8 opcode, UInt8 rtype, UInt8 rid, const UInt8 *data, UInt32 dlen,
                                       UInt8 *rbuf, UInt32 rlen) {
    UInt8 b[16 + 128];
    UInt32 n = 0;
    if (dlen > 128) return kIOReturnBadArgument;
    b[n++] = cmdReg & 0xFF; b[n++] = cmdReg >> 8;
    if (opcode == HID_SET_POWER) b[n++] = rid & 1;          // rid carries the power state here
    else b[n++] = (UInt8)((rtype << 4) | (rid >= 0x0F ? 0x0F : rid));
    b[n++] = opcode;
    if (opcode != HID_SET_POWER && rid >= 0x0F) b[n++] = rid;
    if (opcode == HID_GET_REPORT || opcode == HID_SET_REPORT) { b[n++] = dataReg & 0xFF; b[n++] = dataReg >> 8; }
    if (opcode == HID_SET_REPORT) {
        UInt16 sz = (UInt16)(2 + (rid ? 1 : 0) + dlen);
        b[n++] = sz & 0xFF; b[n++] = sz >> 8;
        if (rid) b[n++] = rid;
        memcpy(b + n, data, dlen); n += dlen;
    }
    return xfer(b, n, rbuf, rlen);
}

bool SimpleI2CTouchpad::readHidDescriptor() {
    UInt8 w[2] = { (UInt8)(descReg & 0xFF), (UInt8)(descReg >> 8) };
    IOReturn r = kIOReturnError;
    for (int k = 0; k < 5; k++) {
        r = xfer(w, 2, hidDesc, sizeof(hidDesc));
        if (r == kIOReturnSuccess) break;
        IOSleep(20);
    }
    if (r != kIOReturnSuccess) { LOG("HID descriptor read failed (0x%x) at address 0x%x reg 0x%x", r, addr, descReg); return false; }
    UInt16 len = OSReadLittleInt16(hidDesc, 0), bcd = OSReadLittleInt16(hidDesc, 2);
    repDescLen = OSReadLittleInt16(hidDesc, 4); repDescReg = OSReadLittleInt16(hidDesc, 6);
    inputReg = OSReadLittleInt16(hidDesc, 8); maxInput = OSReadLittleInt16(hidDesc, 10);
    outputReg = OSReadLittleInt16(hidDesc, 12); cmdReg = OSReadLittleInt16(hidDesc, 16); dataReg = OSReadLittleInt16(hidDesc, 18);
    vid = OSReadLittleInt16(hidDesc, 20); pid = OSReadLittleInt16(hidDesc, 22); ver = OSReadLittleInt16(hidDesc, 24);
    LOG("HID descriptor: len %u bcd 0x%x reportDesc %u@0x%x input 0x%x max %u cmd 0x%x data 0x%x vid 0x%04x pid 0x%04x ver 0x%x",
        len, bcd, repDescLen, repDescReg, inputReg, maxInput, cmdReg, dataReg, vid, pid, ver);
    if (len != 30 || bcd != 0x0100 || !repDescLen || repDescLen > 8192 || maxInput < 3 || maxInput > 4096) {
        hexdump("bad HID descriptor", hidDesc, sizeof(hidDesc));
        return false;
    }
    return true;
}

bool SimpleI2CTouchpad::waitAttention(UInt32 ms) {
    if (!gpioOK) { IOSleep(ms < 100 ? ms : 100); return true; }
    for (UInt32 t = 0; t < ms; t += 2) {
        if (padLevel() == 0) return true;
        IOSleep(2);
    }
    return false;
}

int SimpleI2CTouchpad::readInput() {
    if (!inBuf) return -1;
    memset(inBuf, 0, 2);
    IOReturn r = xfer(nullptr, 0, inBuf, maxInput);
    if (r != kIOReturnSuccess) return -1;
    UInt16 len = OSReadLittleInt16(inBuf, 0);
    if (len == 0 || len == 0xFFFF) return 0;
    if (len < 3 || len > maxInput) {
        if (nErrors++ < 20) LOG("input report with bad length %u (max %u)", len, maxInput);
        return -1;
    }
    return 1;
}

void SimpleI2CTouchpad::setModes() {
    UInt8 buf[128];
    // Input Mode = 3 (precision touchpad reports instead of mouse reports) and, in the same report if it is there,
    // Surface Switch / Button Switch = 1.
    if (L.inputMode.ok) {
        UInt8 rid = L.inputMode.f.rid;
        UInt32 bytes = (parser.featBits[rid] + 7) / 8;
        if (bytes == 0 || bytes > sizeof(buf)) bytes = 1;
        memset(buf, 0, sizeof(buf));
        UInt8 resp[3 + 128];
        if (hidCommand(HID_GET_REPORT, HID_REPORT_FEATURE, rid, nullptr, 0, resp, 2 + (rid ? 1 : 0) + bytes) == kIOReturnSuccess) {
            UInt32 hdr = 2 + (rid ? 1 : 0);
            memcpy(buf, resp + hdr, bytes);
        }
        setField(buf, bytes, L.inputMode.f, 3);
        if (L.surfaceSwitch.ok && L.surfaceSwitch.f.rid == rid) setField(buf, bytes, L.surfaceSwitch.f, 1);
        if (L.buttonSwitch.ok && L.buttonSwitch.f.rid == rid) setField(buf, bytes, L.buttonSwitch.f, 1);
        IOReturn r = hidCommand(HID_SET_REPORT, HID_REPORT_FEATURE, rid, buf, bytes, nullptr, 0);
        LOG("set Input Mode = 3 (report 0x%x, %u bytes): 0x%x", rid, bytes, r);
    } else {
        LOG("no Input Mode feature in the report descriptor - the touchpad may stay in mouse mode");
    }
    if ((L.surfaceSwitch.ok && (!L.inputMode.ok || L.surfaceSwitch.f.rid != L.inputMode.f.rid)) ||
        (L.buttonSwitch.ok && (!L.inputMode.ok || L.buttonSwitch.f.rid != L.inputMode.f.rid))) {
        UInt8 rid = L.surfaceSwitch.ok ? L.surfaceSwitch.f.rid : L.buttonSwitch.f.rid;
        UInt32 bytes = (parser.featBits[rid] + 7) / 8;
        if (bytes == 0 || bytes > sizeof(buf)) bytes = 1;
        memset(buf, 0, sizeof(buf));
        if (L.surfaceSwitch.ok && L.surfaceSwitch.f.rid == rid) setField(buf, bytes, L.surfaceSwitch.f, 1);
        if (L.buttonSwitch.ok && L.buttonSwitch.f.rid == rid) setField(buf, bytes, L.buttonSwitch.f, 1);
        IOReturn r = hidCommand(HID_SET_REPORT, HID_REPORT_FEATURE, rid, buf, bytes, nullptr, 0);
        LOG("set Surface/Button switch = 1 (report 0x%x): 0x%x", rid, r);
    }
}

bool SimpleI2CTouchpad::hidInit(bool first) {
    if (!readHidDescriptor()) return false;
    if (first || !inBuf || inBufLen < (UInt32)maxInput + 4) {
        if (inBuf) IOFree(inBuf, inBufLen);
        inBufLen = maxInput + 4;
        inBuf = (UInt8 *)IOMalloc(inBufLen);
        if (!inBuf) return false;
    }
    // power on (some devices NAK the first command while asleep), then reset and wait for the reset response
    if (hidCommand(HID_SET_POWER, 0, 0, nullptr, 0, nullptr, 0) != kIOReturnSuccess) {
        IOSleep(10);
        hidCommand(HID_SET_POWER, 0, 0, nullptr, 0, nullptr, 0);
    }
    IOSleep(2);
    IOReturn r = hidCommand(HID_RESET, 0, 0, nullptr, 0, nullptr, 0);
    if (r != kIOReturnSuccess) { LOG("RESET command failed 0x%x", r); return false; }
    UInt64 t0 = nowNs();
    bool att = waitAttention(1500);
    int rr = readInput();
    LOG("reset response after %llu ms: attention %s, read %d (len %u)", (nowNs() - t0) / 1000000, att ? "yes" : "TIMEOUT", rr,
        OSReadLittleInt16(inBuf, 0));
    if (gpioOK && !att) LOG("WARNING: the attention line never went low after RESET - check the GPIOPad value");

    // report descriptor
    if (first || !repDesc) {
        if (repDesc) IOFree(repDesc, repDescLen);
        repDesc = (UInt8 *)IOMalloc(repDescLen);
        if (!repDesc) return false;
        UInt8 w[2] = { (UInt8)(repDescReg & 0xFF), (UInt8)(repDescReg >> 8) };
        r = xfer(w, 2, repDesc, repDescLen);
        if (r != kIOReturnSuccess) { LOG("report descriptor read failed 0x%x", r); IOFree(repDesc, repDescLen); repDesc = nullptr; return false; }
        if (debugLevel >= 1) hexdump("report descriptor", repDesc, repDescLen);
        OSData *dd = OSData::withBytes(repDesc, repDescLen);
        if (dd) { setProperty("ReportDescriptor", dd); dd->release(); }
        if (!parser.fields) parser.fields = (HidField *)IOMalloc(sizeof(HidField) * MAX_FIELDS);
        if (!parser.fields) return false;
        if (!parser.parse(repDesc, repDescLen)) { LOG("report descriptor parse error"); return false; }
        if (!parser.layout(L)) { LOG("no precision touchpad report found (%d fields)", parser.nFields); return false; }
        const HidField &fx = L.fingers[0].x.f, &fy = L.fingers[0].y.f;
        minX = fx.lmin; minY = fy.lmin;
        logMaxX = (UInt32)(fx.lmax - fx.lmin); logMaxY = (UInt32)(fy.lmax - fy.lmin);
        physMaxX = physicalMax001mm(fx); physMaxY = physicalMax001mm(fy);
        if (!physMaxX || !physMaxY) { physMaxX = 10500; physMaxY = 7000; }
        LOG("touchpad report 0x%x: %u bytes, %d finger slots, contact count %s, button %s, scan time %s; X %d..%d Y %d..%d, %u.%02u x %u.%02u mm",
            L.rid, L.inBytes, L.nFingers, L.contactCount.ok ? "yes" : "no", L.button.ok ? "yes" : "no", L.scanTime.ok ? "yes" : "no",
            fx.lmin, fx.lmax, fy.lmin, fy.lmax, physMaxX / 100, physMaxX % 100, physMaxY / 100, physMaxY % 100);
        for (int k = 0; k < L.nFingers; k++) {
            const FingerFields &F = L.fingers[k];
            LOG("  finger %d: tip %s conf %s id %s x %s@%u/%u y %s@%u/%u", k, F.tip.ok ? "y" : "n", F.conf.ok ? "y" : "n", F.id.ok ? "y" : "n",
                F.x.ok ? "y" : "n", F.x.f.bitOff, F.x.f.bits, F.y.ok ? "y" : "n", F.y.f.bitOff, F.y.f.bits);
        }
        LOG("features: input mode %s (report 0x%x), surface switch %s, button switch %s, contact max %s, pad type %s",
            L.inputMode.ok ? "yes" : "no", L.inputMode.f.rid, L.surfaceSwitch.ok ? "yes" : "no", L.buttonSwitch.ok ? "yes" : "no",
            L.contactMax.ok ? "yes" : "no", L.padType.ok ? "yes" : "no");
    }
    if (L.contactMax.ok && first) {
        UInt8 rid = L.contactMax.f.rid; UInt32 bytes = (parser.featBits[rid] + 7) / 8;
        UInt8 resp[3 + 64];
        if (bytes && bytes <= 64 &&
            hidCommand(HID_GET_REPORT, HID_REPORT_FEATURE, rid, nullptr, 0, resp, 2 + (rid ? 1 : 0) + bytes) == kIOReturnSuccess) {
            const UInt8 *p = resp + 2 + (rid ? 1 : 0);
            LOG("device: max contacts %d%s%s", getField(p, bytes, L.contactMax.f),
                L.padType.ok && L.padType.f.rid == rid ? ", pad type " : "",
                L.padType.ok && L.padType.f.rid == rid ? (getField(p, bytes, L.padType.f) == 0 ? "clickpad" : "pressure/non-clickable") : "");
        }
    }
    setModes();
    // drain anything queued
    for (int k = 0; k < 8 && (!gpioOK || padLevel() == 0); k++) if (readInput() <= 0) break;
    if (gpioOK) LOG("attention line idle level after init: %d (expected 1)", padLevel());
    return true;
}

void SimpleI2CTouchpad::publish() {
    setProperty(VOODOO_INPUT_LOGICAL_MAX_X_KEY, logMaxX, 32);
    setProperty(VOODOO_INPUT_LOGICAL_MAX_Y_KEY, logMaxY, 32);
    setProperty(VOODOO_INPUT_PHYSICAL_MAX_X_KEY, physMaxX, 32);
    setProperty(VOODOO_INPUT_PHYSICAL_MAX_Y_KEY, physMaxY, 32);
    setProperty(VOODOO_INPUT_TRANSFORM_KEY, 0ULL, 32);
    setProperty("VoodooInputSupported", kOSBooleanTrue);
    char s[64];
    snprintf(s, sizeof(s), "0x%04x:0x%04x", vid, pid);
    setProperty("Touchpad", s);
    setProperty("I2C Address", addr, 16);
    published = true;
    registerService();
    LOG("ready: VoodooInput can attach now");
}

// ---------------------------------------------------------------- frames ----------------------------------------
void SimpleI2CTouchpad::handleReport(const UInt8 *p, UInt32 n) {
    nReports++;
    if (debugLevel >= 2 && nReports <= 150) hexdump("report", p, n);
    UInt8 rid = 0;
    if (L.usesIds) { if (!n) return; rid = p[0]; p++; n--; }
    if (rid != L.rid) {
        if (nForeign++ < 10) LOG("ignoring report id 0x%x (%u bytes) - touchpad report is 0x%x", rid, n, L.rid);
        return;
    }
    lastReportNs = nowNs();
    int cc = L.contactCount.ok ? getField(p, n, L.contactCount.f) : -1;
    bool btn = L.button.ok ? getField(p, n, L.button.f) != 0 : false;
    if (cc > 0 || frameExpected < 0) {
        if (frameExpected > 0 && frameN < frameExpected) {
            nPartial++;
            if (debugLevel >= 1) LOG("incomplete frame: %d of %d contacts - using what arrived", frameN, frameExpected);
            flushFrame();
        }
        frameN = 0;
        frameExpected = (cc >= 0) ? cc : L.nFingers;
        frameButton = btn;
    } else {
        frameButton = frameButton || btn;
    }
    for (int k = 0; k < L.nFingers && frameN < frameExpected && frameN < MAX_FINGERS * 2; k++) {
        const FingerFields &F = L.fingers[k];
        Contact &c = frame[frameN];
        c.tip = F.tip.ok ? getField(p, n, F.tip.f) != 0 : false;
        c.conf = F.conf.ok ? getField(p, n, F.conf.f) != 0 : true;
        c.id = F.id.ok ? (UInt32)getField(p, n, F.id.f) : (UInt32)k;
        c.x = F.x.ok ? getField(p, n, F.x.f) : 0;
        c.y = F.y.ok ? getField(p, n, F.y.f) : 0;
        if (cc < 0 && !c.tip && !(c.x || c.y)) continue; // no contact count: skip empty slots
        frameN++;
    }
    if (frameN >= frameExpected || cc < 0) flushFrame();
}

void SimpleI2CTouchpad::flushFrame() {
    if (frameExpected < 0) return;
    UInt64 t; clock_get_uptime(&t);
    memset(&ev, 0, sizeof(ev));
    bool seen[16] = {};
    int nt = 0, nActive = 0;
    for (int k = 0; k < frameN && nt < VOODOO_INPUT_MAX_TRANSDUCERS; k++) {
        const Contact &c = frame[k];
        int slot = -1;
        for (int s = 0; s < 16; s++) if (slots[s].active && slots[s].id == c.id) { slot = s; break; }
        if (slot < 0) for (int s = 0; s < 16; s++) if (!slots[s].active && !seen[s]) { slot = s; break; }
        if (slot < 0 || seen[slot]) continue; // duplicate id in one frame
        seen[slot] = true;
        SInt32 x = c.x - minX, y = c.y - minY;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if ((UInt32)x > logMaxX) x = (SInt32)logMaxX;
        if ((UInt32)y > logMaxY) y = (SInt32)logMaxY;
        SlotState &st = slots[slot];
        VoodooInputTransducer &tr = ev.transducers[nt];
        tr.timestamp = t;
        tr.type = FINGER;
        tr.isValid = true;
        tr.secondaryId = c.id;
        tr.fingerType = c.conf ? (MT2FingerType)(kMT2FingerTypeIndexFinger + (nt % 4)) : kMT2FingerTypePalm;
        tr.isTransducerActive = c.tip;
        tr.isPhysicalButtonDown = frameButton;
        tr.supportsPressure = false;
        tr.currentCoordinates.x = (UInt32)x; tr.currentCoordinates.y = (UInt32)y;
        tr.previousCoordinates.x = st.active ? st.x : (UInt32)x;
        tr.previousCoordinates.y = st.active ? st.y : (UInt32)y;
        if (c.tip) { st.active = true; st.id = c.id; st.x = (UInt32)x; st.y = (UInt32)y; nActive++; }
        else st.active = false;
        nt++;
    }
    // a finger that was down and is simply missing from this frame: lift it explicitly
    for (int s = 0; s < 16 && nt < VOODOO_INPUT_MAX_TRANSDUCERS; s++) {
        if (!slots[s].active || seen[s]) continue;
        VoodooInputTransducer &tr = ev.transducers[nt++];
        tr.timestamp = t; tr.type = FINGER; tr.isValid = true; tr.secondaryId = slots[s].id;
        tr.fingerType = kMT2FingerTypeIndexFinger; tr.isTransducerActive = false; tr.isPhysicalButtonDown = frameButton;
        tr.currentCoordinates.x = tr.previousCoordinates.x = slots[s].x;
        tr.currentCoordinates.y = tr.previousCoordinates.y = slots[s].y;
        slots[s].active = false;
        nSynthLifts++;
        if (debugLevel >= 1 && nSynthLifts <= 50) LOG("finger %u vanished without a lift - lifted", slots[s].id);
    }
    // thumb: with a click or 4+ fingers, the lowest finger is the thumb (helps click-drag and 4-finger pinch)
    if (nActive >= 4 || frameButton) {
        UInt32 ymax = 0; int th = -1;
        for (int k = 0; k < nt; k++)
            if (ev.transducers[k].isTransducerActive && ev.transducers[k].fingerType != kMT2FingerTypePalm &&
                ev.transducers[k].currentCoordinates.y >= ymax) { ymax = ev.transducers[k].currentCoordinates.y; th = k; }
        if (th >= 0 && (nActive >= 2)) ev.transducers[th].fingerType = kMT2FingerTypeThumb;
    }
    ev.contact_count = (UInt8)nt;
    ev.timestamp = t;
    if (nt == 0) ev.transducers[0].isPhysicalButtonDown = frameButton;
    bool empty = (nActive == 0);
    if (!(nt == 0 && !frameButton && lastSentEmpty && !lastButton)) {
        if (voodooInput) messageClient(kIOMessageVoodooInputMessage, voodooInput, &ev, sizeof(ev));
        nFrames++;
        if (debugLevel >= 1 && nFrames <= 300) {
            LOG("frame %u: %d contacts (%d down) btn %d%s x0 %u y0 %u id0 %u", nFrames, nt, nActive, frameButton,
                voodooInput ? "" : " (no VoodooInput yet)", nt ? ev.transducers[0].currentCoordinates.x : 0,
                nt ? ev.transducers[0].currentCoordinates.y : 0, nt ? ev.transducers[0].secondaryId : 0);
        }
    }
    if (!empty) lastTouchNs = nowNs();
    lastSentEmpty = empty;
    lastButton = frameButton;
    frameExpected = -1; frameN = 0;
}

void SimpleI2CTouchpad::sendLiftAll(const char *why) {
    bool any = lastButton;
    for (int s = 0; s < 16; s++) any |= slots[s].active;
    if (!any) return;
    if (debugLevel >= 1) LOG("lifting all fingers: %s", why);
    frameN = 0; frameExpected = 0; frameButton = false;
    flushFrame();
}

void SimpleI2CTouchpad::updateStats() {
    OSDictionary *d = OSDictionary::withCapacity(10);
    if (!d) return;
    struct { const char *k; UInt32 v; } items[] = {
        { "reports", nReports }, { "frames", nFrames }, { "empty reads", nEmpty }, { "i2c errors", nErrors },
        { "lifts added", nSynthLifts }, { "watchdog lifts", nWatchdog }, { "other reports", nForeign }, { "incomplete frames", nPartial },
    };
    for (auto &it : items) {
        OSNumber *n = OSNumber::withNumber(it.v, 32);
        if (n) { d->setObject(it.k, n); n->release(); }
    }
    setProperty("Stats", d);
    d->release();
}

void SimpleI2CTouchpad::timerFired(IOTimerEventSource *src) {
    UInt32 next = 8;
    switch (state) {
    case kStateInit: {
        if (hidInit(!published)) {
            state = kStateRun;
            initFailures = 0;
            consecutiveEmpty = 0;
            if (!published) publish();
            next = 2;
        } else if (++initFailures >= 6) {
            LOG("touchpad init failed %d times - giving up (VoodooI2C can be re-enabled in config.plist)", initFailures);
            state = kStateDead;
            return;
        } else {
            LOG("touchpad init failed - retry %d in 1 s", initFailures);
            next = 1000;
        }
        break;
    }
    case kStateRun: {
        UInt64 now = nowNs();
        int lvl = padLevel();
        bool pending = gpioOK ? (lvl == 0) : true;
        for (int k = 0; pending && k < 6; k++) {
            int rr = readInput();
            if (rr > 0) {
                consecutiveEmpty = 0;
                handleReport(inBuf + 2, OSReadLittleInt16(inBuf, 0) - 2);
            } else {
                if (rr == 0) nEmpty++;
                if (gpioOK && rr == 0 && ++consecutiveEmpty == 200) {
                    LOG("attention line low but nothing to read 200 times - switching to polling");
                    gpioOK = false;
                    setProperty("Attention", "polling (GPIO level unreliable)");
                }
                break;
            }
            pending = gpioOK ? (padLevel() == 0) : false;
        }
        now = nowNs();
        bool touching = false;
        for (int s = 0; s < 16; s++) touching |= slots[s].active;
        if (watchdogMs && (touching || lastButton) && lastReportNs && now - lastReportNs > (UInt64)watchdogMs * 1000000ULL) {
            nWatchdog++;
            sendLiftAll("touchpad silent - watchdog");
            touching = false;
        }
        bool recent = lastTouchNs && now - lastTouchNs < 1000000000ULL;
        if (gpioOK) next = (touching || recent) ? 2 : 8;
        else next = (touching || recent) ? 6 : 12;
        if (now - lastStatsNs > 5000000000ULL) { lastStatsNs = now; updateStats(); }
        break;
    }
    default:
        return;
    }
    src->setTimeoutMS(next);
}

// ---------------------------------------------------------------- IOService --------------------------------------
IOService *SimpleI2CTouchpad::probe(IOService *provider, SInt32 *score) {
    if (!IOService::probe(provider, score)) return nullptr;
    int dummy[4];
    if (PE_parse_boot_argn("-sitpoff", dummy, sizeof(dummy))) { LOG("disabled by -sitpoff"); return nullptr; }
    pci = OSDynamicCast(IOPCIDevice, provider);
    if (!pci) return nullptr;
    bool ok = findAcpi();
    if (ok) LOG("probe: %s has HID touchpad %s", ctrlAcpi->getName(), devAcpi->getName());
    OSSafeReleaseNULL(ctrlAcpi); OSSafeReleaseNULL(devAcpi);
    pci = nullptr;
    return ok ? this : nullptr;
}

bool SimpleI2CTouchpad::start(IOService *provider) {
    if (!IOService::start(provider)) return false;
    pci = OSDynamicCast(IOPCIDevice, provider);
    if (!pci) return false;
    pci->retain();
    debugLevel = numProp("DebugLevel", 1);
    watchdogMs = numProp("WatchdogMs", 300);
    UInt32 dl = 0;
    if (PE_parse_boot_argn("sitpdebug", &dl, sizeof(dl))) debugLevel = dl;
    LOG("start (version 1.0.0, debug %u)", debugLevel);
    if (!findAcpi()) { LOG("no HID touchpad under this controller"); return false; }
    readAcpiConfig();
    LOG("controller %s, touchpad %s: address 0x%x, %u Hz, HID descriptor reg 0x%x, FS %u/%u SS %u/%u hold 0x%x",
        ctrlAcpi->getName(), devAcpi->getName(), addr, busHz, descReg, fsH, fsL, ssH, ssL, sdaHold);
    if (!pci->open(this)) LOG("could not open the PCI device (continuing)");
    configurePci();
    if (!mapController()) { stop(provider); return false; }
    if (!initBus()) { stop(provider); return false; }
    LOG("bus: DesignWare v%c.%c%c, FIFO tx %u rx %u", (char)(compVer >> 24), (char)(compVer >> 16), (char)(compVer >> 8), txDepth, rxDepth);
    setupGpio();
    if (!gpioOK) setProperty("Attention", "polling");
    memset(slots, 0, sizeof(slots));

    wl = IOWorkLoop::workLoop();
    if (!wl) { stop(provider); return false; }
    timer = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &SimpleI2CTouchpad::timerFired));
    gate = IOCommandGate::commandGate(this);
    if (!timer || !gate || wl->addEventSource(timer) != kIOReturnSuccess || wl->addEventSource(gate) != kIOReturnSuccess) {
        stop(provider); return false;
    }
    PMinit();
    pmStarted = true;
    pci->joinPMtree(this);
    registerPowerDriver(this, gPowerStates, 2);
    state = kStateInit;
    timer->setTimeoutMS(20);
    return true;
}

void SimpleI2CTouchpad::stop(IOService *provider) {
    LOG("stop");
    if (timer) timer->cancelTimeout();
    if (gate) gate->runAction(OSMemberFunctionCast(IOCommandGate::Action, this, &SimpleI2CTouchpad::gatedPower), (void *)2);
    if (pmStarted) { PMstop(); pmStarted = false; }
    if (wl) {
        if (timer) wl->removeEventSource(timer);
        if (gate) wl->removeEventSource(gate);
    }
    OSSafeReleaseNULL(timer); OSSafeReleaseNULL(gate); OSSafeReleaseNULL(wl);
    if (regs) setEnable(false);
    OSSafeReleaseNULL(mmio); regs = nullptr;
    OSSafeReleaseNULL(gpioMap); padDW0 = nullptr;
    if (pci) { if (pci->isOpen(this)) pci->close(this); }
    OSSafeReleaseNULL(pci);
    OSSafeReleaseNULL(ctrlAcpi); OSSafeReleaseNULL(devAcpi);
    IOService::stop(provider);
}

void SimpleI2CTouchpad::free() {
    if (inBuf) { IOFree(inBuf, inBufLen); inBuf = nullptr; }
    if (repDesc) { IOFree(repDesc, repDescLen); repDesc = nullptr; }
    if (parser.fields) { IOFree(parser.fields, sizeof(HidField) * MAX_FIELDS); parser.fields = nullptr; }
    OSSafeReleaseNULL(voodooInput);
    IOService::free();
}

// a0: 0 = sleep, 1 = wake, 2 = stop
IOReturn SimpleI2CTouchpad::gatedPower(void *a0, void *, void *, void *) {
    uintptr_t what = (uintptr_t)a0;
    if (what == 0 || what == 2) {
        if (state == kStateAsleep || state == kStateDead) return kIOReturnSuccess;
        if (timer) timer->cancelTimeout();
        sendLiftAll("sleep");
        if (state == kStateRun && regs) hidCommand(HID_SET_POWER, 0, 1, nullptr, 0, nullptr, 0);
        if (regs) setEnable(false);
        state = kStateAsleep;
        if (what == 0) { OSSafeReleaseNULL(mmio); regs = nullptr; }
        LOG("%s", what == 0 ? "sleep" : "stopped");
    } else {
        if (state != kStateAsleep) return kIOReturnSuccess;
        configurePci();
        if (!mapController() || !initBus()) { LOG("wake: controller did not come back"); state = kStateDead; return kIOReturnSuccess; }
        memset(slots, 0, sizeof(slots));
        frameExpected = -1; frameN = 0; lastButton = false; lastSentEmpty = true;
        state = kStateInit;
        initFailures = 0;
        LOG("wake: re-initialising the touchpad");
        if (timer) timer->setTimeoutMS(10);
    }
    return kIOReturnSuccess;
}

IOReturn SimpleI2CTouchpad::setPowerState(unsigned long ps, IOService *dev) {
    if (dev != this || !gate) return kIOPMAckImplied;
    gate->runAction(OSMemberFunctionCast(IOCommandGate::Action, this, &SimpleI2CTouchpad::gatedPower), (void *)(uintptr_t)(ps ? 1 : 0));
    return kIOPMAckImplied;
}

bool SimpleI2CTouchpad::handleOpen(IOService *client, IOOptionBits, void *) {
    if (client && client->getProperty(VOODOO_INPUT_IDENTIFIER)) {
        OSSafeReleaseNULL(voodooInput);
        voodooInput = client;
        voodooInput->retain();
        LOG("VoodooInput attached");
        return true;
    }
    return false;
}

bool SimpleI2CTouchpad::handleIsOpen(const IOService *client) const {
    return voodooInput && client == voodooInput;
}

void SimpleI2CTouchpad::handleClose(IOService *client, IOOptionBits) {
    if (voodooInput && client == voodooInput) OSSafeReleaseNULL(voodooInput);
}

// kmod record: macOS refuses a kext without _kmod_info
#include <mach/kmod.h>
extern "C" {
kern_return_t _start(kmod_info_t *ki, void *data);
kern_return_t _stop(kmod_info_t *ki, void *data);
KMOD_EXPLICIT_DECL(local.SimpleI2CTouchpad, "1.0.0", _start, _stop)
__private_extern__ kmod_start_func_t *_realmain = 0;
__private_extern__ kmod_stop_func_t *_antimain = 0;
}

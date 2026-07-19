from opendbc.car.structs import CarParams
from opendbc.car.fisker.values import CAR

Ecu = CarParams.Ecu

# Message-based fingerprint for the Fisker Ocean, extracted from a real 1-minute ADAS-bus (can0 /
# openpilot bus 0) capture. Every message seen on bus 0 (addr < 0x800) must be listed with its
# correct length or fingerprinting eliminates the platform, so this is the complete observed bus-0
# set (173 messages), including the stock ADAS ECU's control messages (0x121 ADAS_ACCEL_CONTROL,
# 0x1D0 ADAS_STEER_CONTROL). Diagnostic addresses (0x7df/0x7e0/0x7e8) are excluded to match
# opendbc/car/car_helpers.py:can_fingerprint. TODO: some low-rate messages may still be missing.
FINGERPRINTS = {
  CAR.FISKER_OCEAN: [{
    32: 8, 54: 8, 83: 8, 89: 8, 237: 8, 258: 8, 274: 8, 275: 8, 276: 8, 277: 8, 278: 8, 279: 8, 280: 8,
    281: 8, 288: 8, 289: 8, 293: 8, 298: 8, 336: 8, 337: 8, 345: 8, 368: 8, 372: 8, 373: 8, 374: 8, 375: 8,
    376: 8, 377: 8, 438: 8, 440: 8, 442: 8, 448: 8, 450: 8, 452: 8, 464: 8, 510: 8, 522: 8, 523: 8, 524: 8,
    525: 8, 526: 8, 527: 8, 528: 8, 532: 16, 537: 8, 549: 8, 588: 32, 592: 8, 593: 8, 594: 8, 597: 8, 616: 8,
    678: 8, 684: 8, 711: 8, 714: 8, 717: 8, 720: 8, 723: 8, 726: 8, 729: 8, 732: 8, 735: 8, 738: 8, 741: 8,
    744: 8, 745: 8, 746: 8, 767: 8, 778: 8, 784: 8, 785: 8, 787: 8, 788: 8, 789: 8, 790: 8, 791: 8, 792: 8,
    794: 8, 795: 8, 796: 8, 801: 8, 811: 8, 813: 8, 815: 8, 817: 16, 818: 8, 819: 8, 820: 8, 821: 8, 822: 8,
    825: 8, 827: 8, 829: 8, 831: 8, 832: 8, 834: 8, 835: 8, 837: 8, 843: 8, 845: 8, 847: 8, 848: 8, 849: 8,
    850: 8, 851: 8, 854: 8, 855: 8, 856: 8, 857: 8, 859: 8, 865: 8, 866: 8, 868: 8, 873: 8, 875: 8, 877: 8,
    883: 8, 891: 16, 896: 8, 899: 8, 903: 8, 989: 8, 1065: 8, 1069: 8, 1086: 8, 1137: 8, 1141: 8, 1153: 8, 1154: 8,
    1159: 8, 1170: 8, 1264: 8, 1268: 16, 1287: 8, 1297: 8, 1300: 8, 1310: 8, 1317: 8, 1318: 8, 1319: 8, 1321: 8, 1322: 8,
    1329: 8, 1410: 8, 1476: 16, 1552: 8, 1583: 8, 1625: 8, 1654: 8, 1662: 8, 1664: 8, 1671: 8, 1673: 8, 1674: 8, 1760: 8,
    1765: 8, 1766: 8, 1767: 8, 1921: 8, 1922: 8, 1929: 8, 1930: 8, 1940: 8, 1942: 8, 1948: 8, 1950: 8, 1954: 8, 1958: 8,
    1959: 8, 1962: 8, 1966: 8, 1967: 8,
  }],
}

# TODO: populate once ECU FW versions are captured via UDS (0xF188/0xF191) on the DIAG bus
FW_VERSIONS: dict = {}

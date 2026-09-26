from opendbc.car.structs import CarParams
from opendbc.car.fisker.values import CAR

Ecu = CarParams.Ecu


# CAN fingerprint — every non-diagnostic address the Fisker Ocean broadcasts on ADASBUS
# (Bus.pt = 0) with the car in Run mode. openpilot's can_fingerprint uses an EXCLUSIVE
# match (see opendbc.car.fingerprints.is_valid_for_fingerprint): the candidate is kept
# only if every observed CAN address < 0x800 is present in this dict with a matching DLC,
# so a minimal "distinctive-only" fingerprint doesn't work — the FULL bus contents are
# required.
#
# Note: this fingerprint reflects one trim/firmware. If a different trim adds or removes
# any address, the exclusive match fails — extend the address set or switch to FW-based
# identification if that becomes an issue.
FINGERPRINTS = {
  CAR.FISKER_OCEAN: [{
    0x020: 8,  0x036: 8,  0x053: 8,  0x059: 8,  0x0ED: 8,  0x102: 8,
    0x112: 8,  0x113: 8,  0x114: 8,  0x115: 8,  0x116: 8,  0x117: 8,
    0x118: 8,  0x119: 8,  0x120: 8,  0x121: 8,  0x125: 8,  0x12A: 8,
    0x150: 8,  0x151: 8,  0x159: 8,  0x170: 8,  0x174: 8,  0x175: 8,
    0x176: 8,  0x177: 8,  0x178: 8,  0x179: 8,  0x1B6: 8,  0x1B8: 8,
    0x1BA: 8,  0x1C0: 8,  0x1C2: 8,  0x1C4: 8,  0x1D0: 8,  0x1FE: 8,
    0x20A: 8,  0x20B: 8,  0x20C: 8,  0x20D: 8,  0x20E: 8,  0x20F: 8,
    0x210: 8,  0x214: 16, 0x219: 8,  0x225: 8,  0x24C: 32, 0x250: 8,
    0x251: 8,  0x252: 8,  0x255: 8,  0x268: 8,  0x2A6: 8,  0x2AC: 8,
    0x2C7: 8,  0x2CA: 8,  0x2CD: 8,  0x2D0: 8,  0x2D3: 8,  0x2D6: 8,
    0x2D9: 8,  0x2DC: 8,  0x2DF: 8,  0x2E2: 8,  0x2E5: 8,  0x2E8: 8,
    0x2E9: 8,  0x2EA: 8,  0x2FF: 8,  0x30A: 8,  0x310: 8,  0x311: 8,
    0x313: 8,  0x314: 8,  0x315: 8,  0x316: 8,  0x317: 8,  0x318: 8,
    0x31A: 8,  0x31B: 8,  0x31C: 8,  0x321: 8,  0x32B: 8,  0x32D: 8,
    0x32F: 8,  0x331: 16, 0x332: 8,  0x333: 8,  0x334: 8,  0x335: 8,
    0x336: 8,  0x339: 8,  0x33B: 8,  0x33D: 8,  0x33F: 8,  0x340: 8,
    0x342: 8,  0x343: 8,  0x345: 8,  0x348: 8,  0x34B: 8,  0x34D: 8,
    0x34F: 8,  0x350: 8,  0x351: 8,  0x352: 8,  0x353: 8,  0x356: 8,
    0x357: 8,  0x358: 8,  0x359: 8,  0x35B: 8,  0x361: 8,  0x362: 8,
    0x364: 8,  0x369: 8,  0x36B: 8,  0x36D: 8,  0x373: 8,  0x37B: 16,
    0x380: 8,  0x383: 8,  0x387: 8,  0x3DD: 8,  0x429: 8,  0x42D: 8,
    0x42E: 8,  0x43E: 8,  0x471: 8,  0x475: 8,  0x481: 8,  0x482: 8,
    0x487: 8,  0x492: 8,  0x4F0: 8,  0x4F4: 16, 0x507: 8,  0x511: 8,
    0x513: 8,  0x514: 8,  0x51E: 8,  0x525: 8,  0x526: 8,  0x527: 8,
    0x529: 8,  0x52A: 8,  0x531: 8,  0x582: 8,  0x5C4: 16, 0x610: 8,
    0x62F: 8,  0x659: 8,  0x676: 8,  0x67E: 8,  0x680: 8,  0x687: 8,
    0x689: 8,  0x68A: 8,  0x6E0: 8,  0x6E5: 8,  0x6E6: 8,  0x6E7: 8,
  },{
    1788: 64, 275: 8, 278: 8, 288: 8, 1789: 64, 438: 8, 778: 8, 442: 8,
    532: 16, 258: 8, 1268: 16, 537: 8, 549: 8, 448: 8, 279: 8, 280: 8
, 464: 8, 289: 8, 785: 8, 787: 8, 788: 8, 856: 8, 767: 8, 678: 8, 1297: 8,
684: 8, 281: 8, 450: 8, 368: 8, 452: 8, 237: 8, 811: 8, 336: 8, 855: 8,
89: 8, 337: 8, 1287: 8, 293: 8, 274: 8, 298: 8, 813: 8, 815: 8, 276: 8,
845: 8, 277: 8, 440: 8, 616: 8, 792: 8, 784: 8, 829: 8, 843: 8, 847: 8,
1317: 8, 827: 8, 831: 8, 345: 8, 1069: 8, 817: 16, 1300: 8, 883: 8, 526: 8,
527: 8, 796: 8, 832: 8, 1410: 8, 522: 8, 524: 8, 528: 8, 789: 8, 790: 8,
791: 8, 818: 8, 523: 8, 795: 8, 820: 8, 54: 8, 834: 8, 1476: 16, 525: 8,
794: 8, 825: 8, 801: 8, 819: 8, 821: 8, 835: 8, 837: 8, 868: 8, 873: 8,
1310: 8, 372: 8, 373: 8, 374: 8, 375: 8, 376: 8, 377: 8, 83: 8, 1318: 8,
896: 8, 1170: 8, 1264: 8, 510: 8, 891: 16, 875: 8, 989: 8, 1159: 8, 723: 8,
726: 8, 746: 8, 848: 8, 899: 8, 1141: 8, 1319: 8, 711: 8, 714: 8, 849: 8,
857: 8, 717: 8, 732: 8, 735: 8, 741: 8, 822: 8, 850: 8, 1153: 8, 1154: 8,
720: 8, 729: 8, 738: 8, 1321: 8, 1329: 8, 744: 8, 745: 8, 851: 8, 854: 8,
588: 32, 859: 8, 592: 8, 593: 8, 594: 8, 865: 8, 597: 8, 866: 8, 1137: 8,
1322: 8, 903: 8, 1552: 8, 1070: 8, 32: 8, 1583: 8, 1654: 8, 877: 8, 1086: 8,
1065: 8, 1674: 8, 1662: 8, 1664: 8, 1671: 8, 1673: 8, 1625: 8
}],
}


# UDS FW-version fingerprints. We do not actually query FW versions on the Ocean —
# FW_QUERY_CONFIG.requests is empty, so no UDS traffic is generated at runtime — but
# match_fw_to_car_exact eliminates a brand candidate by walking its ECU list and
# failing when a live FW doesn't match; with a truly empty ECU dict, the walk is
# skipped and fisker becomes a "matches everything" false-positive against other
# brands' fingerprints (see test_exact_match). This placeholder essential ECU at a
# fisker-unique address ensures the exact matcher can reject fisker for non-Ocean
# live FW versions. Replace with the real EPS FW string once one is captured on
# vehicle; the address 0x7A5 is not queried by any brand today.
FW_VERSIONS: dict = {
  CAR.FISKER_OCEAN: {
    (Ecu.eps, 0x7A5, None): [b'FISKER_OCEAN_PLACEHOLDER'],
  },
}


# WMI (World Manufacturer Identifier) prefixes for VIN-based coarse identification.
FISKER_WMI = {"VCF"}

from bluepy.btle import Scanner, DefaultDelegate, ScanEntry
# import config as cfg
from time import time
from math import isinf
HEADSET_ADDR = "cc:98:8b:cf:bc:82"
FOLLOWED_PHONE_MAC = "0c:e4:a0:b4:ab:54"
ALTBEACON = "7e:e9:98:a8:15:2d"

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.rssi = 0
        self.prevRssi = 0
        self.start = 0.0
        self.end = 0.0
        self.started = False

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if (dev.addr == ALTBEACON) :
            self.rssi = dev.rssi

            # if self.prevRssi != self.rssi and self.started :
            #     self.end = time()
            #     print("bt scan time", self.end - self.start)
            #     self.started = False
            # elif not self.started :
            #     self.start = time()
            #     self.started = True
            if self.started :
                self.end = time()
                print("bt scan time", self.end - self.start)
                self.started = False
            elif not self.started :
                self.start = time()
                self.started = True
            self.prevRssi = self.rssi
            print(dev.rssi)
            # print("found device to folow!!! <3: ", self.rssi, "devRssi:", dev.rssi)
        if isNewDev:
            print ("Discovered device", dev.addr)
        elif isNewData:
            # if (self.started) :
            #     self.end = time()
            #     self.started = False
            #     print ("scan interval : ", end - start)
            # else :
            #     self.start = time()
            #     self.started = True
            # print ("new rssi : ", dev.rssi)
            print ("Received new data from", dev.addr)
            


class DeviceScanner(Scanner) :
    def __init__(self, scanDelegate) :
        Scanner().__init__(self)
        # self.scanDelegate = ScanDelegate()
        self.scanner = Scanner().withDelegate(scanDelegate)
        self.rssi = scanDelegate.rssi
        self.devices = []

    def startScan(self, scanTime=float("inf")) :
        if not isinf(scanTime) :
            self.devices = self.scanner.scan(scanTime)
        else :
            self.scanner.start()
            while True:
                print ("BLE scan running...")
                self.scanner.process(timeout=5)

    def showAvilableDevices(self, scanDelegate) :
        self.rssi = scanDelegate.rssi
        print("SCANDELEGATE RSSI", scanDelegate.rssi)
        for dev in self.devices:
            print ("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
            for (adtype, desc, value) in dev.getScanData():
                print ("  %s = %s" % (desc, value))
            print ("###########NEXT DEVICE################")



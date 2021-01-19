from bluepy.btle import Scanner, DefaultDelegate, ScanEntry
from peopleFollower import config as cfg

from math import isinf
HEADSET_ADDR = "cc:98:8b:cf:bc:82"
FOLLOWED_PHONE_MAC = "0c:e4:a0:b4:aB:54"

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.rssi = 0

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print ("Discovered device", dev.addr)
            if (dev.addr == HEADSET_ADDR) :
                self.rssi = dev.rssi
                print("found device to folow!!! <3: ", self.rssi, "devRssi:", dev.rssi)
        elif isNewData:
            print ("Received new data from", dev.addr)


class DeviceScanner(Scanner) :
    def __init__(self, scanDelegate) :
        Scanner().__init__(self)
        # self.scanDelegate = ScanDelegate()
        self.scanner = Scanner().withDelegate(scanDelegate)
        self.rssi = scanDelegate.rssi
        self.devices = []

    def startScan(self, scanTime=10.0) :
        if not isinf(scanTime) :
            self.devices = self.scanner.scan(scanTime)
        else :
            self.scanner.start()
            while True:
                print ("BLE scan running...")
                self.scanner.process()

    def showAvilableDevices(self, scanDelegate) :
        self.rssi = scanDelegate.rssi
        print("SCANDELEGATE RSSI", scanDelegate.rssi)
        for dev in self.devices:
            print ("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
            for (adtype, desc, value) in dev.getScanData():
                print ("  %s = %s" % (desc, value))
            print ("###########NEXT DEVICE################")



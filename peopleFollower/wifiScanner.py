from wifi import Cell, Scheme
from rssi import RSSI_Scan, RSSI_Localizer
# from utils import findRssiDistance
from time import sleep, time

import config as cfg


# RSSI_Scan(interface='wlan0 ').getAPinfo()

class WifiScanner () :
	def __init__(self) :
		# self.wifilist = []
		self.MY_SSID = 'HUAWEI P30 Pro'
		self.interface = 'wlan0'

	def updateRssi(self) :
		cell = self.FindFromSearchList(self.MY_SSID)
		if cell != False :
			cfg.rssi = cell.signal

	def showMyRssi(self) :
		print(self.MY_SSID, cfg.rssi)

	def probeRssi(self, probingFreq=100) :
		try:
			while not cfg.threadStopper.is_set() :
				self.updateRssi()
				self.showMyRssi()
				sleep(1/probingFreq)
			return cfg.rssi
		except Exception as err:
			print(str(err))
			cfg.threadStopper.set()
		finally:
			cfg.GPG.stop()

	# def Search(self) :
	# 	wifilist = []

	# 	cells = Cell.all(self.interface)
	# 	for cell in cells:
	# 		if cell != None :
	# 			wifilist.append(cell)
	# 	return wifilist


	def FindFromSearchList(self, ssid):
		# wifilist = self.Search()
		# print (wifilist)
		start = time()
		cells = Cell.all(self.interface)
		end = time()
		print (end - start)
		if cells != None :
			for cell in cells:
				if cell.ssid == ssid:
					return cell
		return False

def main() :
	wifiScanner = WifiScanner()
	wifiScanner.probeRssi(1000000)


if __name__ == "__main__":
    # execute only if run as a script
    main()


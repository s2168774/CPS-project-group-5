import ble_robot
import runRobotModes
from threading import Thread

from multiprocessing import Process

# def f(name):
#     print('hello', name)

# if __name__ == '__main__':
#     p = Process(target=f, args=('bob',))
#     p.start()
#     p.join()
# bluetoothThread = Thread(target=ble_robot.main())
bleProcess = Process(target=ble_robot.main)
programLoopThread = Thread(target= runRobotModes.main)

try:
	# bluetoothThread(target=main())
	programLoopThread.start()
	bleProcess.start()
	# ble_robot.main()
	# bluetoothThread.start()

	# print("after ble thread")
	pass
except Exception as e:
	print(str(e))
	programLoopThread.join()
	bleProcess.join()
	raise
	pass
finally:

	pass

# bluetoothThread(target=main())
# programLoopThread.start()
# ble_robot.main()
# bluetoothThread.start()

# print("after ble thread")

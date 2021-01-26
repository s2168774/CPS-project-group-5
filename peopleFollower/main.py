import ble_robot
import runRobotModes
from threading import Thread

# bluetoothThread = Thread(target=ble_robot.main())


programLoopThread = Thread(target= runRobotModes.main())

# bluetoothThread(target=main())
programLoopThread.start()
ble_robot.main()
# bluetoothThread.start()

print("after ble thread")

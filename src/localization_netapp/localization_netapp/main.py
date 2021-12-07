import rclpy
from .evolvedApi.cellid_node import CellidNode
from .evolvedApi import endpoint as ep
from multiprocessing import Process
import uvicorn
import time


def main():
    rclpy.init(args=None)
    proc = Process(target=uvicorn.run,
                        args=(ep.app,),
                        kwargs={
                            "host": "0.0.0.0"},
                        daemon=True)
    proc.start()
    time.sleep(2)
    node = CellidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

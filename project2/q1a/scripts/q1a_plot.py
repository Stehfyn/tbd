import rospy
import sys
from ScanContext.ScanContext import ScanContext

def main():
    sc = ScanContext()
    
    if (len(sys.argv) >= 2):
        sc.deserialize(sys.argv[1])
    else:
        sc.deserialize("LidarScanContext.txt")

    sc.plot_point_cloud()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
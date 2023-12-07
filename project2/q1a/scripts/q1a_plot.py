import sys
import os
from ScanContext.ScanContext import ScanContext


currentdir = os.path.dirname(os.path.abspath(__file__))

def main():
    sc = ScanContext()
    
    if (len(sys.argv) >= 2):
        sc.deserialize(sys.argv[1])
    else:
        sc.deserialize(f"{currentdir}/LidarScanContext.txt")

    sc.plot_point_cloud(write_to_disk=True, path=currentdir)

if __name__ == '__main__':
    
    try:
        main()
    except:
        pass
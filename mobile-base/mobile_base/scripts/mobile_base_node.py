#!/usr/bin/env python
import rospy
import sys
from mobile_base import MOBILE_BASE

def main(args):
  rospy.init_node('mobile_base', anonymous=True)
  mobile_base = MOBILE_BASE()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
#!/usr/bin/env python3
import rospy
from datetime import datetime
import pytz
from pvlib.location import Location
from std_msgs.msg import Float64MultiArray

def get_sun_position(latitude, longitude, timezone):
    location = Location(latitude, longitude, timezone)
    now = datetime.now(pytz.timezone(timezone))
    solpos = location.get_solarposition(now)

    azimuth = solpos['azimuth'].values[0]
    elevation = solpos['elevation'].values[0]
    return azimuth, elevation

def main():
    rospy.init_node('solar_position_node')
    pub = rospy.Publisher('/sun_position', Float64MultiArray, queue_size=10)

    latitude = rospy.get_param('~latitude', 45.815399)
    longitude = rospy.get_param('~longitude', 15.966568)
    timezone = rospy.get_param('~timezone', 'Europe/Zagreb')

    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        az, el = get_sun_position(latitude, longitude, timezone)
        msg = Float64MultiArray()
        msg.data = [az, el]
        pub.publish(msg)
        rospy.loginfo(f"Sun position: azimuth={az:.2f}, elevation={el:.2f}")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

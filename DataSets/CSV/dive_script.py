import numpy as np
import matplotlib.pyplot as plt
import csv

from rosbags.rosbag2 import Reader

from rosbag_types import SmarcRosbagTypestore

def save_to_csv(data:dict):
    for key, val in data.items():
       with open(f"{key}.csv", 'w', newline=f'\n') as csvfile:
           datawriter= csv.writer(csvfile, delimiter=',')
           datawriter.writerows(val)

def main():
    smarc_types = SmarcRosbagTypestore()
    typestore = smarc_types.construct_custom_typestore()
    outputs = {}
    outputs['vbs'] = []
    outputs['thrustervertical'] = []
    outputs['fluid_pressure'] = []
    outputs['dvl'] = []
    outputs['imu'] = []
    outputs['odom'] = []
    outputs['depth'] = []
    with Reader('/home/tko/colcon_ws/src/smarc2/rosbag2_2024_12_09-22_21_01') as reader:
           # Topic and msgtype information is available on .connections list.
           for connection in reader.connections:
               print(connection.topic, connection.msgtype)
           for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/sam_auv_v1/ctrl/conv/control_input':
                   msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                   outputs['vbs'].append([timestamp, msg.vbs])
                   outputs['thrustervertical'].append([timestamp, msg.thrustervertical])
                elif connection.topic == '/sam_auv_v1/core/depth20_pressure':
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    outputs['fluid_pressure'].append([timestamp, msg.fluid_pressure])
                    outputs['depth'].append([timestamp, (msg.fluid_pressure - 101325.0) / 9806.65 ])
                elif connection.topic == '/sam_auv_v1/core/dvl':
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    outputs['dvl'].append([timestamp, msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.altitude])
                elif connection.topic == '/sam_auv_v1/core/imu':
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    outputs['imu'].append([timestamp, msg.linear_acceleration.z])
                elif connection.topic == '/sam_auv_v1/core/odom_gt':
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    position = msg.pose.pose.position
                    outputs['odom'].append([timestamp, position.x,position.y, position.z])
                # elif connection.topic == '/tf':
                #     msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                #     transform = msg.transforms[0]
                #     translation = transform.transform.translation
                #     print(translation.x)
                #     print(translation.y)
                #     print(translation.z)
                #     outputs['odom'].append([timestamp, translation.x,translation.y, translation.z])

    # exit()
    for key, val in outputs.items():
        outputs[key] = np.array(val)

    plt.title('Depth')
    plt.plot(outputs['depth'][:,0], outputs['depth'][:,1])
    plt.savefig('depth.png')
    plt.show()

    plt.plot(outputs['odom'][:,0], outputs['odom'][:,-1], label='z')
    plt.plot(outputs['odom'][:,0], outputs['odom'][:,-2], label='y')
    plt.plot(outputs['odom'][:,0], outputs['odom'][:,-3], label='x')
    plt.title('odom')
    plt.legend()
    plt.savefig('odom.png')
    plt.show()

    plt.plot(outputs['odom'][:,0], outputs['odom'][:,-1], label='z')
    plt.plot(outputs['depth'][:,0], -outputs['depth'][:,1], label='depth')
    plt.title('Depth vs Odom')
    plt.legend()
    plt.savefig('odom.png')
    plt.show()

    plt.plot(outputs['imu'][:,0], outputs['imu'][:,-1])
    plt.title('IMU Z')
    plt.savefig('imu.png')
    plt.show()

    plt.plot(outputs['dvl'][:,0], outputs['dvl'][:,-2],label='z')
    plt.plot(outputs['dvl'][:,0], outputs['dvl'][:,-3],label='y')
    plt.plot(outputs['dvl'][:,0], outputs['dvl'][:,-4],label='x')
    plt.legend()
    plt.title('DVL Components')
    plt.savefig('dvl.png')
    plt.show()

    plt.title('VBS Command')
    plt.plot(outputs['vbs'][:,0], outputs['vbs'][:,1])
    plt.savefig('vbs.png')
    plt.show()

    plt.title('Fluid Pressure')
    plt.plot(outputs['fluid_pressure'][:,0], outputs['fluid_pressure'][:,1])
    plt.savefig('pressure.png')
    plt.show()

    save_to_csv(outputs)
if __name__ == "__main__":
    main()


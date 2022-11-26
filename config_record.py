import configparser as cf

config = cf.ConfigParser()


def config_write():
    config["DEFAULT"] = {'robot_name': 'qy_robot'
                         }
    config['websocket_message'] = {'ip': '192.168.229.183',
                                   'port': '9090'}
    config['personal_msg'] = {'battery': 0}
    config['param'] = {'max_vel_x': 0.0,
                       'min_vel_x': 0.0,
                       'max_vel_trans': 0.0,
                       'min_vel_trans': 0.0,
                       'max_vel_theta': 0.0,
                       'min_vel_theta': 0.0,
                       'sim_time': 0.0,
                       'inflation_radius': 0.0}
    with open('config.ini', 'w') as f:
        config.write(f)


def config_read(flag):
    config.read('config.ini')
    if flag == 1:
        return config['websocket_message']['ip']
    elif flag == 2:
        return config['websocket_message']['port']
    elif flag == 3:
        return config['personal_msg']['battery']


if __name__ == '__main__':
    config_write()
    # print(type(config_read(1)))

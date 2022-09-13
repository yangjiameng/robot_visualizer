import configparser as cf

config = cf.ConfigParser()


def config_write():
    config["DEFAULT"] = {'robot_name': 'qy_robot'
                         }
    config['websocket_message'] = {'ip': '192.168.229.183',
                                   'port': '9090'}
    config['personal_msg'] = {'battery': 0}
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

import sys
import pedroclient

def main(id_):
    client = pedroclient.PedroClient()
    client.register('sensor_'+id_)
    server_address = 'sensor_server'+id_+'@localhost'
    msg = input('Enter a sensor term or q for quit ? ').strip()
    while msg != 'q':
        client.p2p(server_address, msg)
        msg = input('Enter a sensor term or q for quit ? ').strip()







if __name__ == "__main__":
    argc = len(sys.argv)
    if argc > 1:
        id_ = sys.argv[1]
    else:
        id_ = '1'

    main(id_)

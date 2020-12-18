import json
import serial
import os, pickle
import time, hmac, hashlib
import requests

# file settings
label = 'j'
#json_samples = 128 # switch
json_samples = 192 # advanced
path = 'data/'

# Edge Impulse settings
HMAC_KEY = ""

# firmware settings
#capture_samples = 192 # normal
capture_samples = 256 # large
freq = 295
interval = 1000/freq


def capture_serial():
    s = serial.Serial('COM3', baudrate=115200)
    if not os.path.isdir(path + label):
        os.mkdir(path + label)
    i = 0
    print("Capturing label:", label)
    while True:
        arr = []
        j = 0
        while j < capture_samples/4:
            read = s.readline().decode().rstrip().split(',')
            arr.append(list(map(int, read)))
            j = j + 1
        i = i + 1
        print("Captured sample", i)
        fname = label + '.' + str(int(time.time()*1000)) + '.' + str(i)
        pickle.dump(arr, open(path + label + '/' + fname + '.pkl', 'wb'))


def export_json(arr, root, name):
    # empty signature (all zeros). HS256 gives 32 byte signature, and we encode in hex, so we need 64 characters here
    emptySignature = ''.join(['0'] * 64)

    data = {
        "protected": {
            "ver": "v1",
            "alg": "HS256",
            "iat": time.time() # epoch time, seconds since 1970
        },
        "signature": emptySignature,
        "payload": {
            "device_name": "2A:68:65:37:E0:6A:9D:38",
            "device_type": "MANUAL-CAPTURE",
            "interval_ms": interval,
            "sensors": [
                { "name": "u", "units": "N/A" },
                { "name": "d", "units": "N/A" },
                { "name": "l", "units": "N/A" },
                { "name": "r", "units": "N/A" }
            ],
            "values": arr[:int(json_samples/4)]
        }
    }

    # encode in JSON
    encoded = json.dumps(data)

    # sign message
    signature = hmac.new(bytes(HMAC_KEY, 'utf-8'), msg = encoded.encode('utf-8'), digestmod = hashlib.sha256).hexdigest()

    # set the signature again in the message, and encode again
    data['signature'] = signature
    encoded = json.dumps(data)


def convert_json(data_dir):
    for root, dirs, files in os.walk(data_dir):
        for f in files:
            if f.endswith('.pkl'):
                print(os.path.join(root, f))
                arr = pickle.load(open(os.path.join(root, f), 'rb'))
                export_json(arr, root, f)


if __name__ == "__main__":
    capture_serial()
    #convert_json(path)

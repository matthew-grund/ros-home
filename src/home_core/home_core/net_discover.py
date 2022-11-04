# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
from ipaddress import ip_interface
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import scapy.all as scapy
import socket
from netifaces import interfaces, ifaddresses, AF_INET
from mac_vendor_lookup import MacLookup, BaseMacLookup
import requests
import pyamaha
import json
import xmltodict
import time
import sys

class NetworkDeviceDiscoverer(Node):

    def __init__(self):
        super().__init__('network_discover')
        self.publisher_devices = self.create_publisher(String, '/devices/discovered/network', 10)
        self.timer_period = 17.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.JSON_HEADERS = {'Content-type': 'application/json', 'Accept': 'text/plain'}
        self.i = 0
        self.devices=[]
        self.prev_devices=[]        
        self.mac = MacLookup()
        self.wan_ip = "Unknown"
        self.data_folder = "/data/home_ws"
        self.device_cache_path = "/data/home_ws/device-cache.json"
        BaseMacLookup.cache_path = "/data/home_ws/mac-cache.dat"
        # mac.update_vendors()  # <- This can take a few seconds for the download and it will be stored in the new path
        scapy.conf.verb = 0 
        self.ssid = "Unknown"
        self.need_mac_vendor_update = True       
        # self.mac.update_vendors()

    def timer_callback(self):
        self.devices = self.discover_home()
        msg = String()
        m = {}
        m['index'] = self.i
        m['interval'] = self.timer_period
        m['payload'] = self.devices
        msg.data = json.dumps(m)
        self.publisher_devices.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def update_mac_vendor_table(self):
        self.mac.update_vendors()

    def vendor_from_mac(self, mac_address):
        if self.need_mac_vendor_update:
            self.mac.update_vendors()
            self.need_mac_vendor_update = False
        vendor_name = self.mac.lookup(mac_address)
        return vendor_name

    def discover(self, ip):
        arp_req_frame = scapy.ARP(pdst=ip)
        broadcast_ether_frame = scapy.Ether(dst="ff:ff:ff:ff:ff:ff")
        broadcast_ether_arp_req_frame = broadcast_ether_frame / arp_req_frame
        answered_list = scapy.srp(broadcast_ether_arp_req_frame, timeout=1, verbose=False)[0]
        result = []
        for i in range(0, len(answered_list)):
            client_dict = {"ip": answered_list[i][1].psrc, "mac": answered_list[i][1].hwsrc}
            result.append(client_dict)
        return result

    def host_wan_ip(self):
        if self.wan_ip == "Unknown":
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip=s.getsockname()[0]
            s.close()
            self.wan_ip = ip
        return self.wan_ip

    def discover_network_interfaces(self):
        self.host_net_iflist = []
        for ifaceName in interfaces():
            addresses = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr':'No IP addr'}] )]
            if addresses != ['No IP addr']:
                if addresses != ['127.0.0.1']:
                    if addresses != ['192.168.56.1']:
                        if_dict = {"name":ifaceName, "addresses":addresses}
                        self.host_net_iflist.append(if_dict)
        return self.host_net_iflist

    def host_ip_to_scan_ip(self, host_ip):
        (subnet,dot,ip)=host_ip.rpartition(".")
        scan_ip = subnet + ".1/24" # 0 to 255 on subnet of IP passed in
        return scan_ip

    def discover_network_addresses(self):
        host_ip = self.host_wan_ip()
        scanip = self.host_ip_to_scan_ip(host_ip)
        addresses=[]
        ifaces = self.discover_network_interfaces()
        for if_iter in ifaces:
            ip = if_iter['addresses'][0]
            scanip = self.host_ip_to_scan_ip(ip)
            scanres = self.discover(scanip)
            for res in scanres:
                addresses.append(res)
        return addresses

    def tcp_port_is_open(self, dst_ip, dst_port):
        my_ip = self.host_wan_ip()
        ip = scapy.IP(src=my_ip, dst=dst_ip)
        syn_packet = ip / scapy.TCP(sport=1500, dport=dst_port, flags="S", seq=100)
        syn_ack = scapy.sr1(syn_packet,timeout=1)
        if syn_ack is None:
            return False
        elif (syn_ack.haslayer(scapy.TCP)):
            if syn_ack.getlayer(scapy.TCP).flags == 0x12:
                send_rst = scapy.sr1(scapy.IP(dst=dst_ip)/scapy.TCP(sport=1500,dport=dst_port,flags="AR"),timeout=3)
                return True
            elif syn_ack.getlayer(scapy.TCP).flags == 0x14:
                return False
        return False

    def udp_port_is_open(self, dst_ip,dst_port):
        udp_scan_resp = scapy.sr1(scapy.IP(dst=dst_ip)/scapy.UDP(dport=dst_port),timeout=1)
        if udp_scan_resp is None:
            retrans = []
            for count in range(0,3):
                retrans.append(scapy.sr1(scapy.IP(dst=dst_ip)/scapy.UDP(dport=dst_port),timeout=1))
            for item in retrans:
                if item is None:
                    self.udp_port_is_open(dst_ip,dst_port)
            return True
        elif (udp_scan_resp.haslayer(scapy.UDP)):
            return True
        elif(udp_scan_resp.haslayer(scapy.ICMP)):
            if(int(udp_scan_resp.getlayer(scapy.ICMP).type)==3 and int(udp_scan_resp.getlayer(scapy.ICMP).code)==3):
                return False
        elif(int(udp_scan_resp.getlayer(scapy.ICMP).type)==3 and int(udp_scan_resp.getlayer(scapy.ICMP).code) in [1,2,9,10,13]):
            return True
        return False

    def fetch_google_config(self, ip):
        """ Fetch device eureka info. """
        get_url =  ( "http://" + ip + ":" + "8008" + 
            "/setup/eureka_info?options=detail&param=version,build_info,name,detail,device_info,info" )
        try:    
            req = requests.get(get_url, headers=self.JSON_HEADERS, timeout=15.0)
        except:
            return {}
        if req.status_code == 200:    
            eureka = json.loads(req.text)
        else:
            eureka = {}    
        return eureka

    def fetch_yamaha_config(self, ip):
        """ Fetch yamaha device info. """
        get_url =  ( "http://" + ip + ":" + "49154" + "/MediaRenderer/desc.xml" )
        try:    
            ret = requests.get(get_url)
        except:
            return {} # FIXME - a better error value?

        if ret.status_code == 200:      # success! 
            yamaha = xmltodict.parse(ret.text)
            yamaha = yamaha['root']['device']
            if "iconList" in yamaha:
                icon_list = yamaha['iconList']['icon']
                max_width = 0
                max_icon = {}
                for icon in icon_list:
                    if int(icon['width']) > max_width:
                        max_icon = icon
                        max_width = int(icon['width'])
                yamaha['device_icon_url'] =  "http://" + ip + ":" + "49154" + max_icon['url']
                yamaha.pop('iconList')
            if "serviceList" in yamaha:
                yamaha.pop('serviceList')    
            return yamaha
        return {}

    def version(self):
        return "Homer 0.4"
    
    def ssid(self):
        return self.ssid

    def identify_device(self, device):
        # indentify with 
        # identify with MAC based vendors
        try:
            device['vendor'] = self.vendor_from_mac(device['mac'])
        except:
            device['vendor'] = "Unknown"

        if device['vendor'] == "Arcadyan Corporation":   # yamaha receivers, others?
            device = self.identify_arcadyan_device(device)
            if 'is_known' not in device:
                device['is_known'] = False        
        
        elif device['vendor'] == 'BSH Hausgeräte GmbH':  # bosch home connect
            device = self.identify_bosch_device(device)
            if 'is_known' not in device:
                device['is_known'] = False                    

        elif device['vendor'] == 'Google, Inc.': # chromecasts and google homes
            device = self.identify_google_device(device)
            if 'is_known' not in device:
                device['is_known'] = False  

        elif device['vendor'] == 'LG Innotech': # LG  TVs
            device = self.identify_lg_device(device)
            if 'is_known' not in device:
                device['is_known'] = False  

        elif device['vendor'] == 'Microsoft Corporation': # xboxen, etc.
            device = self.identify_microsoft_device(device)
            if 'is_known' not in device:
                device['is_known'] = False  

        elif device['vendor'] == 'Nest Labs Inc.':
            device = self.identify_nest_device(device)
            if 'is_known' not in device:
                device['is_known'] = False              

        elif device['vendor'] =='Technicolor CH USA Inc.':   # lots of routers and modems and DirectTV
            device = self.identify_technicolor_device(device)
            if 'is_known' not in device:
                device['is_known'] = False  

        elif device['vendor'] == 'Texas Instruments':
            device = self.identify_ti_device(device)
            if 'is_known' not in device:
                device['is_known'] = False  

        elif device['vendor'] == 'Wyze Labs Inc':
            device = self.identify_wyze_device(device)
            if 'is_known' not in device:
                device['is_known'] = False  

        else:
            device['type'] = 'Unknown'
            device['has_bluetooth'] = False
            device['name'] = 'Unknown'   #FIXME 
            device['model'] = 'Unknown'
            device['is_known'] = False

        return device
    
    def identify_arcadyan_device(self, device):    
        yamaha = self.fetch_yamaha_config(device['ip'])
        if len(yamaha) > 0:
            device['vendor'] = yamaha['manufacturer']   
            device['type'] = yamaha['modelDescription']
            device['name'] = yamaha['friendlyName'] 
            device['model'] = yamaha['modelName']
            device['serial_number'] = yamaha['serialNumber']
            device['device_icon_url'] = yamaha['device_icon_url']
            device['has_bluetooth'] = True
            device['is_known'] = True
        else:  
            device['type'] = 'Arcadyan' 
            device['name'] = 'Unknown' 
            device['model'] = 'Unknown'


        return device

    def identify_bosch_device(self, device):
        device['type'] = 'Bosch Appliance'   
        device['name'] = 'Unknown'   #FIXME 
        device['model'] = 'Unknown'
        return device
    
    def identify_google_device(self, device):    
        device['type'] = 'Google Chromecast Device'
        eureka = self.fetch_google_config(device['ip'])
        if len(eureka):
            device['name'] = eureka['name']
            device['vendor_config'] = eureka 
            device['is_known'] = True
            device['has_bluetooth'] = True
        device['model'] = 'Unknown'
        return device

    def identify_microsoft_device(self, device):
        device['type'] = 'XBox'      # FIXME - could be another device type            
        device['name'] = 'Unknown'   # FIXME - communicate with device to inspect device info
        device['model'] = 'Unknown'
        device['has_bluetooth'] = False        
        return device

    def identify_nest_device(self, device):
        device['type'] = 'Nest Device'   
        device['name'] = 'Unknown' 
        device['model'] = 'Unknown'
        device['product'] = 'Unknown'
        return device

    def identify_technicolor_device(self, device):
        device['type'] = 'WiFi Router'   
        device['name'] = self.ssid   # FIXME - is this the best 'name'?
        device['model'] = 'Unknown'
        return device
    
    def identify_ti_device(self, device):
        yamaha = self.fetch_yamaha_config(device['ip'])
        if len(yamaha) > 0:
            device['vendor'] = yamaha['manufacturer']   
            device['type'] = yamaha['modelDescription']
            device['name'] = yamaha['friendlyName'] 
            device['model'] = yamaha['modelName']
            device['serial_number'] = yamaha['serialNumber']
            device['device_icon_url'] = yamaha['device_icon_url']
            device['has_bluetooth'] = True  
            device['is_known'] = True
        else:  
            device['type'] = 'Texas Instruments'
            device['name'] = 'Unknown' 
            device['model'] = 'Unknown'
        return device
    
    def identify_wyze_device(self, device):
        device['type']='Wyze Cam'    # FIXME - could be another device type            
        device['name'] = 'Unknown'   # FIXME - communicate with device to inspect device info
        device['model'] = 'Unknown'  
        return device

    def discover_home(self):
        self.prev_devices = self.devices 
        self.devices = self.discover_network_addresses()
        for device in self.devices:
            device = self.identify_device(device) 
        return self.devices

def main(args=None):
    rclpy.init(args=args)

    home_discover = NetworkDeviceDiscoverer()

    rclpy.spin(home_discover)

    home_discover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

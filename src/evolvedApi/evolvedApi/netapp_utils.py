from evolved5g import swagger_client
from evolved5g.swagger_client import LoginApi, User
from evolved5g.swagger_client.models import Token
import requests, json
import os
import configparser 
import json

class Utils:
    def __init__(self) -> None:
        self.get_configs()

    def get_configs(self):
        config = configparser.RawConfigParser()
        configFilePath = '/evolved5g/cfg/netapp.properties'
        try:
            config.read(configFilePath)
        except Exception as e :
            print(str(e))
        self.nef_username = config.get("configs", "nef_user")
        self.nef_pass = config.get("configs", "nef_pass")
        self.nef_ip = os.environ.get("NEF_IP")
        self.nef_port = os.environ.get("NEF_PORT")
        self.ue_external_id = os.environ.get("UE_EXTERNAL_ID")
        self.netapp_ip = os.environ.get("NETAPP_IP")
        self.netapp_port = os.environ.get("NETAPP_PORT")
        self.netapp_id = config.get("configs", "netapp_id")
        self.capif_callback_ip = config.get("configs", "capif_callback_ip")
        self.capif_callback_port = config.get("configs", "capif_callback_port")
        self.certificates_folder = config.get("configs", "certificates_folder")
        self.capif_host = os.environ.get("CAPIF_HOST")
        self.capif_https_port = os.environ.get("CAPIF_HTTPS_PORT")

        print("================================================")
        print("Starting NetApp with the following configuration:")
        print("================================================")
        print("NEF_IP:", self.nef_ip)
        print("UE_EXTERNAL_ID:", self.ue_external_id)
        print("NETAPP_IP:", self.netapp_ip)
        print("NETAPP_PORT:", self.netapp_port)
        print("NETAPP_ID:", self.netapp_id)
        print("CERTIFICATES_FOLDER:", self.certificates_folder)
        print("CAPIF_HOST:", self.capif_host)
        print("CAPIF_HTTPS_PORT:", self.capif_https_port)
        print("================================================")

    def get_token(self) -> Token:
        configuration = swagger_client.Configuration()
        configuration.host = self.get_host_of_the_nef_emulator()
        api_client = swagger_client.ApiClient(configuration=configuration)
        api_client.select_header_content_type(["application/x-www-form-urlencoded"])
        api = LoginApi(api_client)
        token = api.login_access_token_api_v1_login_access_token_post(
            "", self.nef_username, self.nef_pass, "", "", ""
        )
        print("NEF Token: {}\n".format(token.access_token))
        return token


    def get_api_client(token,self) -> swagger_client.ApiClient:
        configuration = swagger_client.Configuration()
        configuration.host = self.get_host_of_the_nef_emulator()
        configuration.access_token = token.access_token
        api_client = swagger_client.ApiClient(configuration=configuration)
        return api_client


    def get_host_of_the_nef_emulator(self) -> str:
        return "http://{}:{}".format(self.nef_ip, self.nef_port)

    def get_host_of_the_netapp(self) -> str:
        return "http://{}:{}/monitoring/callback".format(self.netapp_ip, self.netapp_port)

def read_cellid() -> int:
    cellid_path = (
        "http://localhost:" + os.environ.get("NETAPP_PORT") + "/cellid"
    )
    response = requests.get(cellid_path, headers=None, data=None)

    return response.json()



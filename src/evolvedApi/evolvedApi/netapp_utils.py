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
        configFilePath = "/evolved5g/cfg/netapp.properties"
        try:
            config.read(configFilePath)
        except Exception as e:
            print(str(e))
        self.nef_username = os.environ.get("NEF_USER")
        self.nef_pass = os.environ.get("NEF_PASSWORD")
        self.nef_address = os.environ.get("NEF_ADDRESS")
        self.nef_port = os.environ.get("NEF_PORT")
        self.ue_external_id_1 = os.environ.get("UE_EXTERNAL_ID_1")
        self.ue_external_id_2 = os.environ.get("UE_EXTERNAL_ID_2")
        self.callback_address = os.environ.get("CALLBACK_ADDRESS")
        self.netapp_id = config.get("configs", "netapp_id")
        self.capif_callback_ip = config.get("configs", "capif_callback_ip")
        self.capif_callback_port = config.get("configs", "capif_callback_port")
        self.certificates_folder = os.environ.get("PATH_TO_CERTS")
        self.capif_host = os.environ.get("CAPIF_HOSTNAME")
        self.capif_https_port = os.environ.get("CAPIF_PORT_HTTPS")

        print("================================================")
        print("Starting NetApp with the following configuration:")
        print("================================================")
        print("NEF_ADDRESS:", self.nef_address)
        print("NEF_PORT:", self.nef_port)
        print("UE_EXTERNAL_ID_1:", self.ue_external_id_1)
        print("UE_EXTERNAL_ID_2:", self.ue_external_id_2)
        print("CALLBACK_ADDRESS:", self.callback_address)
        print("NETAPP_ID:", self.netapp_id)
        print("CERTIFICATES_FOLDER:", self.certificates_folder)
        print("CAPIF_HOST:", self.capif_host)
        print("CAPIF_HTTPS_PORT:", self.capif_https_port)
        print("================================================")

    def get_token(self) -> Token:
        configuration = swagger_client.Configuration()
        configuration.host = self.get_host_of_the_nef_emulator()
        configuration.verify_ssl = False
        api_client = swagger_client.ApiClient(configuration=configuration)
        api_client.select_header_content_type(["application/x-www-form-urlencoded"])
        api = LoginApi(api_client)
        print("Trying to get NEF token on nef host:", configuration.host)
        token = api.login_access_token_api_v1_login_access_token_post(
            "", self.nef_username, self.nef_pass, "", "", ""
        )
        print("NEF Token: {}\n".format(token.access_token))
        return token

    def get_api_client(token, self) -> swagger_client.ApiClient:
        configuration = swagger_client.Configuration()
        configuration.host = self.get_host_of_the_nef_emulator()
        configuration.access_token = token.access_token
        api_client = swagger_client.ApiClient(configuration=configuration)
        return api_client

    def get_host_of_the_nef_emulator(self) -> str:
        return "https://{}:{}".format(self.nef_address, self.nef_port)

    def get_host_of_the_netapp(self) -> str:
        return "http://{}/monitoring/callback".format(self.callback_address)


def read_cellid_user_1() -> int:
    cellid_path = "http://localhost:8000" + "/cellid_user_1"
    response = requests.get(cellid_path, headers=None, data=None)

    return response.json()


def read_cellid_user_2() -> int:
    cellid_path = "http://localhost:8000" + "/cellid_user_2"
    response = requests.get(cellid_path, headers=None, data=None)

    return response.json()

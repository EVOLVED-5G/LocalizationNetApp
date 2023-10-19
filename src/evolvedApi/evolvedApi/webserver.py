from fastapi import FastAPI, Request, Depends
from pydantic import BaseModel, IPvAnyAddress, AnyHttpUrl, Field
from typing import Optional
from enum import Enum
from fastapi_utils.cbv import cbv
from fastapi_utils.inferring_router import InferringRouter
import evolvedApi.netapp_utils as netapp_utils


class LocationInfo(BaseModel):
    cellId: Optional[str] = None
    enodeBId: Optional[str] = None


class MonitoringType(str, Enum):
    locationReporting = "LOCATION_REPORTING"
    lossOfConnectivity = "LOSS_OF_CONNECTIVITY"


class MonitoringEventReport(BaseModel):
    externalId: Optional[str] = Field(
        "123456789@domain.com",
        description="Globally unique identifier containing a Domain Identifier and a Local Identifier. \<Local Identifier\>@\<Domain Identifier\>",
    )
    monitoringType: MonitoringType
    locationInfo: Optional[LocationInfo] = None
    ipv4Addr: Optional[IPvAnyAddress] = Field(
        None, description="String identifying an Ipv4 address"
    )


class MonitoringNotification(MonitoringEventReport):
    subscription: AnyHttpUrl


# Creation of the endpoint using FastAPI
# Here you receive the callback notification from the emulator

app = FastAPI()
netapp_router = InferringRouter()  # Step 1: Create a router

utils = netapp_utils.Utils()
cellid_user_1 = 1
cellid_user_2 = 1


@cbv(netapp_router)
class netapp_endpoint:
    @netapp_router.post("/monitoring/callback")
    def fill_cellid(self, item: MonitoringNotification, request: Request):
        global cellid_user_1, cellid_user_2
        print("[webserver] Received cell_id change")
        print(item.externalId)
        print(item.locationInfo.cellId)
        if item.externalId == utils.ue_external_id_1:
            print("[webserver] Received cell_id change for user 1")
            print(item.locationInfo.cellId)
            cellid_user_1 = int(item.locationInfo.cellId[-1])
        if item.externalId == utils.ue_external_id_2:
            print("[webserver] Received cell_id change for user 2")
            print(item.locationInfo.cellId)
            cellid_user_2 = int(item.locationInfo.cellId[-1])
        return {"ack": "TRUE"}

    @netapp_router.get("/cellid_user_1")
    def get_cellid_user_1(self, request: Request):
        global cellid_user_1
        return cellid_user_1

    @netapp_router.get("/cellid_user_2")
    def get_cellid_user_2(self, request: Request):
        global cellid_user_2
        return cellid_user_2


app.include_router(netapp_router)

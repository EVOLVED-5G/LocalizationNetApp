from fastapi import FastAPI, Request, Depends
from pydantic import BaseModel, IPvAnyAddress, AnyHttpUrl, Field
from typing import Optional
from enum import Enum
from fastapi_utils.cbv import cbv
from fastapi_utils.inferring_router import InferringRouter


class LocationInfo(BaseModel):
    cellId: Optional[str] = None
    enodeBId: Optional[str] = None


class MonitoringType(str, Enum):
    locationReporting = "LOCATION_REPORTING"
    lossOfConnectivity = "LOSS_OF_CONNECTIVITY"


class MonitoringEventReport(BaseModel):
    externalId: Optional[str] = Field("123456789@domain.com",
            description="Globally unique identifier containing a Domain Identifier and a Local Identifier. \<Local Identifier\>@\<Domain Identifier\>")
    monitoringType: MonitoringType
    locationInfo: Optional[LocationInfo] = None
    ipv4Addr: Optional[IPvAnyAddress] = Field(None, description="String identifying an Ipv4 address")


class MonitoringNotification(MonitoringEventReport):
    subscription: AnyHttpUrl


# Creation of the endpoint using FastAPI
# Here you receive the callback notification from the emulator

app = FastAPI()
netapp_router = InferringRouter()  # Step 1: Create a router
cellid = 1

@cbv(netapp_router)
class netapp_endpoint:

    @netapp_router.post("/monitoring/callback")
    def fill_cellid(self, item: MonitoringNotification, request: Request):
        global cellid
        cellid = int(item.locationInfo.cellId[-1])
        return {'ack': 'TRUE'}

    @netapp_router.get("/cellid")
    def get_cellid(self,request: Request):
        global cellid
        return cellid


app.include_router(netapp_router)

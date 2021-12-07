import requests, json
from datetime import datetime, timedelta


class SimulatorAPI(object):
    """
        This class provides a python API wrapper for the NEF simulator
    """

    def __init__(self):
        """
        Save login token
        """
        self.token = self.access()

    def access(self) -> str:
        """
        Login in the simulator
        :return: login token
        """
        response = requests.post("http://localhost:8888/api/v1/login/access-token", data=
        {
            'username': 'admin@my-email.com',
            'password': 'pass'
        })

        jsonfile = response.json()
        return jsonfile['access_token']

    def read_cellid(self, user_id) -> int:
        """
        retrieve the closest cell id given the user
        :param user_id: string of user application id
        :return: return the closest cell id
        """
        header = {}
        response = requests.get('http://0.0.0.0:8000/cellid',
                                headers=None,
                                data=None
                                )

        return response.json()

    def location_subscription(self, external_id):
        """
        retrieve the closest cell id given the user
        :param user_id: string of user application id
        :return: return the closest cell id
        """

        now = datetime.now()
        tomorrow = timedelta(days=+1)
        expiretime = now + tomorrow

        header = {'Authorization': 'Bearer ' + self.token}
        response = requests.post(
            'http://localhost:8888/api/v1/3gpp-monitoring-event/v1/LocalizationNetApp/subscriptions',
            headers=header,
            data=json.dumps({
                "externalId": external_id,
                "notificationDestination": 'http://host.docker.internal:8000/monitoring/callback',
                "monitoringType": "LOCATION_REPORTING",
                "maximumNumberOfReports": 100,
                "monitorExpireTime": expiretime.isoformat()
            })
        )

        return response.json()

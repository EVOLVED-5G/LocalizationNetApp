import requests


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
        header = {'Authorization': 'Bearer ' + self.token}
        response = requests.get('http://localhost:8888/api/v1/UEs/' + user_id,
                                headers=header,
                                data={'supi': user_id}
                                )

        return response.json()['Cell_id']

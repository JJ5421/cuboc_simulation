class BaseController:

    def __init__(self, config):

        self.config = config

    def update(
        self,
        robot,
        world,
        dt
    ):

        raise NotImplementedError(
            "Controllers must implement update()."
        )
from SettingsValues import SettingsValues as SV


class Settings:
    def __init__(self):
        self.settings = {'screen_mode': SV.FULL_SCREEN}
        self._new_settings = {'screen_mode': SV.FULL_SCREEN}

    def update_settings(self):
        self.settings = self._new_settings

    def get_settings(self):
        return self.settings

    def set_window_mode(self, _, setting):
        self._new_settings['screen_mode'] = setting

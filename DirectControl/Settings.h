#pragma once

class Settings {
public:
    void SetOptimalDefaults();
    Settings();
    ~Settings() = default;

    void ReadSettings(const char *file);

    bool TrackShowDebug;                        // to default show track line
};

extern Settings gSettings;

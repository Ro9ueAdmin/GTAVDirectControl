#pragma once
#include "inc/natives.h"
#include <string>
#include "inc/enums.h"


class BlipX {
public:
    BlipX(Entity entity) : mEntity(entity) {
        mHandle = UI::ADD_BLIP_FOR_ENTITY(entity);
    }

    Vector3 GetPosition() const {
        return UI::GET_BLIP_INFO_ID_COORD(mHandle);
    }

    void SetPosition(Vector3 coord) {
        UI::SET_BLIP_COORDS(mHandle, coord.x, coord.y, coord.z);
    }

    eBlipColor GetColor() const {
        return (eBlipColor)UI::GET_BLIP_COLOUR(mHandle);
    }

    void SetColor(eBlipColor color) {
        UI::SET_BLIP_COLOUR(mHandle, color);
    }

    int GetAlpha() const {
        return UI::GET_BLIP_ALPHA(mHandle);
    }

    void SetAlpha(int alpha) {
        UI::SET_BLIP_ALPHA(mHandle, alpha);
    }

    void SetRotation(int rotation) {
        UI::SET_BLIP_ROTATION(mHandle, rotation);
    }

    void SetScale(float scale) {
        UI::SET_BLIP_SCALE(mHandle, scale);
    }

    eBlipSprite GetSprite() const {
        return (eBlipSprite)UI::GET_BLIP_SPRITE(mHandle);
    }

    void SetSprite(eBlipSprite sprite) {
        UI::SET_BLIP_SPRITE(mHandle, sprite);
    }

    Entity GetEntity() const {
        return mEntity;// UI::GET_BLIP_INFO_ID_ENTITY_INDEX(mHandle);
    }

    void SetName(std::string name) {
        UI::BEGIN_TEXT_COMMAND_SET_BLIP_NAME("STRING");
        UI::ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME((char*)name.c_str());
        UI::END_TEXT_COMMAND_SET_BLIP_NAME(mHandle);
    }

    void Delete() {
        UI::SET_BLIP_ALPHA(mHandle, 0);
        UI::REMOVE_BLIP(&mHandle);
    }

    bool Exists() const {
        return UI::DOES_BLIP_EXIST(mHandle);
    }

    Blip GetHandle() const {
        return mHandle;
    }

private:
    int mHandle;
    Entity mEntity;
};

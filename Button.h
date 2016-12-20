/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Button.h
 * Author: pi
 *
 * Created on 07 July 2016, 03:50
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <string>
#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include "Common.h"
#include <iostream>
#include <SDL.h>

using namespace std;

class Button
{
    private:
    //The attributes of the button
    SDL_Rect box;

    //The part of the button sprite sheet that will be shown
    SDL_Rect *mClips;
    SDL_Rect *clip;

    //[Do.Tuan] Add ID, Name variable
    private:
            int id;
            string btnName;
            SDL_Surface* buttonImg;
            NAV_CMD_TYPE delegateCMD;
	
    public:
    //Initialize the variables
    Button( int x, int y, int w, int h, SDL_Surface* image, NAV_CMD_TYPE cmd);
	Button( int x, int y, int w, int h, SDL_Surface* image, NAV_CMD_TYPE cmd, int _id, string _name);

    //Handles events and set the button's sprite region
    bool handle_events(SDL_Event event);

    //Set clip
    void setClips(SDL_Rect *clips);
    
    //Shows the button on the screen
    void show(SDL_Surface* screen);
    void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip = NULL );
	
    //[Do.Tuan] Set and Get ID & Name

    int setButtonIdAndName(int _id, string _name = "")
    {
            id = _id;
            btnName = _name;
    }

    int getButtonId()
    {
            return id;
    }

    string getButtonName()
    {
            return btnName;
    }

    SDL_Surface* getButtonImage()
    {
            return buttonImg;
    }
	
};

#endif
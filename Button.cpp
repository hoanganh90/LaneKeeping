/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Button.cpp
 * Author: pi
 * 
 * Created on 07 July 2016, 03:50
 */

#include "Button.h"
#include <sys/time.h>
#include <iostream> 
#include "Vision.h"

const int CLIP_MOUSEOVER = 0;
const int CLIP_MOUSEOUT = 1;
const int CLIP_MOUSEDOWN = 2;
const int CLIP_MOUSEUP = 3;

Button::Button( int x, int y, int w, int h, SDL_Surface* image, NAV_CMD_TYPE cmd )
{
    //Set the button's attributes
    box.x = x;
    box.y = y;
    box.w = w;
    box.h = h;
	buttonImg = image;
	delegateCMD = cmd;

    //Set the default sprite
    //clip = &mClips[ CLIP_MOUSEOUT ];
}

Button::Button( int x, int y, int w, int h, SDL_Surface* image, NAV_CMD_TYPE cmd, int _id, string _name)
{
    //Set the button's attributes
    box.x = x;
    box.y = y;
    box.w = w;
    box.h = h;
	buttonImg = image;
	delegateCMD = cmd;
	id = _id;
	btnName = _name;
	
    //Set the default sprite
    //clip = &mClips[ CLIP_MOUSEOUT ];
}

void Button::setClips(SDL_Rect *clips)
{
    mClips = clips;
    clip = &mClips[ CLIP_MOUSEOUT ];
}

bool Button::handle_events(SDL_Event event)
{
    //The mouse offsets
    int x = 0, y = 0;
	bool bIsInButtonArea = false;
	
    //If the mouse moved
    if( event.type == SDL_MOUSEMOTION )
    {
        //Get the mouse offsets
        x = event.motion.x;
        y = event.motion.y;

        //If the mouse is over the button
        if( ( x > box.x ) && ( x < box.x + box.w ) && ( y > box.y ) && ( y < box.y + box.h ) )
        {
            //Set the button sprite
            clip = &mClips[ CLIP_MOUSEOVER ];
			bIsInButtonArea = true;
        }
        //If not
        else
        {
            //Set the button sprite
            clip = &mClips[ CLIP_MOUSEOUT ];
        }
    }
    //If a mouse button was pressed
    if( event.type == SDL_MOUSEBUTTONDOWN )
    {
        //If the left mouse button was pressed
        if( event.button.button == SDL_BUTTON_LEFT )
        {
            //Get the mouse offsets
            x = event.button.x;
            y = event.button.y;

            //If the mouse is over the button
            if( ( x > box.x ) && ( x < box.x + box.w ) && ( y > box.y ) && ( y < box.y + box.h ) )
            {
                //Set the button sprite
                clip = &mClips[ CLIP_MOUSEDOWN ];
				bIsInButtonArea = true;
            }
        }
    }
    //If a mouse button was released
    if( event.type == SDL_MOUSEBUTTONUP )
    {
        //If the left mouse button was released
        if( event.button.button == SDL_BUTTON_LEFT )
        {
            //Get the mouse offsets
            x = event.button.x;
            y = event.button.y;

            //If the mouse is over the button
            if( ( x > box.x ) && ( x < box.x + box.w ) && ( y > box.y ) && ( y < box.y + box.h ) )
            {
                //Set the button sprite
                clip = &mClips[ CLIP_MOUSEUP ];
                bIsInButtonArea = true;

                if (delegateCMD == Auto)
                {		
                    mNavigationCommand = Forward;
                    //cong.anh
                    isAutoMode = true;
                }
                else if (delegateCMD == Manual)
                {
                    mNavigationCommand = Stop;
                    //cong.anh
                    isAutoMode = false;
                }
                //cong.anh
                else if (delegateCMD == Land)
                {
                    isAutoMode = false;
                    isLandModeClicked = true;
                    mNavigationCommand = delegateCMD;
                }
                //
                else
                {
                    mNavigationCommand = delegateCMD;
                }
                cout << btnName << " clicked" << endl;
            }
        }
    }
	
	return bIsInButtonArea;
}

void Button::show(SDL_Surface* screen)
{
    //Show the button
    apply_surface( box.x, box.y, buttonImg, screen, clip );
}

void Button::apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination, SDL_Rect* clip)
{
    //Holds offsets
    SDL_Rect offset;

    //Get offsets
    offset.x = x;
    offset.y = y;

    //Blit
    SDL_BlitSurface( source, clip, destination, &offset );
}


/*
 * SDL.h
 *
 *  Created on: Nov 10, 2016
 *      Author: DoThanhTuan
 */

#ifndef MYSDL_H_
#define MYSDL_H_

#include <string.h>
#include "Button.h"
#include "SDL/SDL.h"
#include "SDL/SDL_image.h"

const int NUMBER_OF_BUTTONS = 12;

//The button states in the sprite sheet
const int CLIP_MOUSEOVER = 0;
const int CLIP_MOUSEOUT = 1;
const int CLIP_MOUSEDOWN = 2;
const int CLIP_MOUSEUP = 3;
    
class MySDL
{
public:
    MySDL();
    virtual ~MySDL();

    SDL_Rect clips[ 4 ];
    Button* btnList[NUMBER_OF_BUTTONS];

    //EXTERN SDL_Surface *buttonSheet = NULL;
    SDL_Surface *screen;
 
    bool init();
    SDL_Surface* load_image( std::string filename );
    SDL_Surface* load_image_file(std::string fileName);
    void clean_up(SDL_Surface* btnImage);
    void set_clips();

    //For showing on UI
    bool init_buttons(int numberOfRow, int numberOfColumn, int widthSize, int heightSize);
    bool init_Button_Event_Handler(SDL_Event event);
    bool showButtons();
    void cleanup();

    void fillScreen();
    int update() { return SDL_Flip( screen ); }
};

#endif /* MYSDL_H_ */
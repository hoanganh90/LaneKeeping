/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "MySDL.h"

static string image_path = "buttons/";

static string imgNameList[] = {"Arm.png", "Takeoff.png", "Land.png", "Stop.png",
                                "Auto.png", "Manual.png", "Backward.png", "Forward.png",
                                "Left.png", "Right.png", "CounterClockwise.png", "Clockwise.png"};

static string btnNameList[] = {"Arm", "Take off", "Land", "Stop",
                                "Auto", "Manual", "Backward", "Forward",
                                "Left", "Right", "CounterClockwise", "Clockwise"};

const int SCREEN_WIDTH = 120*4;
const int SCREEN_HEIGHT = 90*3;
const int SCREEN_BPP = 32;

MySDL::MySDL() {
    // TODO Auto-generated constructor stub
}

MySDL::~MySDL() {
    // TODO Auto-generated destructor stub
}
bool MySDL::init()
{
    //Initialize all SDL subsystems
    if( SDL_Init( SDL_INIT_EVERYTHING ) == -1 )
    {
        return false;
    }

    //Set up the screen
    screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );

    //If there was an error in setting up the screen
    if( screen == NULL )
    {
        return false;
    }

    //Set the window caption
    SDL_WM_SetCaption( "Drone Controller", NULL );

    //If everything initialized fine
    return true;
}

SDL_Surface* MySDL::load_image_file(std::string fileName)
{
	SDL_Surface *buttonSheet = NULL;

    //Load the button sprite sheet
	buttonSheet = load_image( fileName );

    //If there was a problem in loading the button sprite sheet
    if( buttonSheet == NULL )
    {
        return NULL;
    }

    //If everything loaded fine
    return buttonSheet;
}

SDL_Surface* MySDL::load_image( std::string filename )
{
    //The image that's loaded
    SDL_Surface* loadedImage = NULL;

    //The optimized surface that will be used
    SDL_Surface* optimizedImage = NULL;

    //Load the image
    loadedImage = IMG_Load( filename.c_str() );

    //If the image loaded
    if( loadedImage != NULL )
    {
        //Create an optimized surface
        optimizedImage = SDL_DisplayFormat( loadedImage );

        //Free the old surface
        SDL_FreeSurface( loadedImage );

        //If the surface was optimized
        if( optimizedImage != NULL )
        {
            //Color key surface
            SDL_SetColorKey( optimizedImage, SDL_SRCCOLORKEY, SDL_MapRGB( optimizedImage->format, 0, 0xFF, 0xFF ) );
        }
    }

    //Return the optimized surface
    return optimizedImage;
}

void MySDL::clean_up(SDL_Surface* btnImage)
{
    //Free the surface
    SDL_FreeSurface( btnImage );

    //Quit SDL
    SDL_Quit();
}

void MySDL::set_clips()
{
    //Clip the sprite sheet
    clips[ CLIP_MOUSEOVER ].x = 0;
    clips[ CLIP_MOUSEOVER ].y = 0;
    clips[ CLIP_MOUSEOVER ].w = 120;
    clips[ CLIP_MOUSEOVER ].h = 90;

    clips[ CLIP_MOUSEOUT ].x = 120;
    clips[ CLIP_MOUSEOUT ].y = 0;
    clips[ CLIP_MOUSEOUT ].w = 120;
    clips[ CLIP_MOUSEOUT ].h = 90;

    clips[ CLIP_MOUSEDOWN ].x = 0;
    clips[ CLIP_MOUSEDOWN ].y = 90;
    clips[ CLIP_MOUSEDOWN ].w = 120;
    clips[ CLIP_MOUSEDOWN ].h = 90;

    clips[ CLIP_MOUSEUP ].x = 120;
    clips[ CLIP_MOUSEUP ].y = 90;
    clips[ CLIP_MOUSEUP ].w = 120;
    clips[ CLIP_MOUSEUP ].h = 90;
}

//For showing UI

bool MySDL::init_buttons(int numberOfRow, int numberOfColumn, int widthSize, int heightSize)
{
    int btnWidth = widthSize / numberOfColumn;
    int btnHeight = heightSize / numberOfRow;
    int btnID = 0;
    int x = 0, y = 0;

    for(int i = 0; i < numberOfRow; i++)
    {
        for(int j = 0; j < numberOfColumn; j++)
        {
            if(btnID < NUMBER_OF_BUTTONS)
            {
                x = j * btnWidth;
                y = i * btnHeight;
                NAV_CMD_TYPE cmd = (NAV_CMD_TYPE) btnID;
                btnList[btnID] = new Button(x, y, btnWidth, btnHeight, load_image(image_path + imgNameList[btnID]), cmd, btnID, btnNameList[btnID]);
                btnList[btnID]->setClips(clips);
                btnID++;
            }
        }
    }
}

bool MySDL::init_Button_Event_Handler(SDL_Event event)
{
    for(int i = 0; i < NUMBER_OF_BUTTONS; i++)
    {
        btnList[i]->handle_events(event);
    }
}

bool MySDL::showButtons()
{
    for(int i = 0; i < NUMBER_OF_BUTTONS; i++)
    {
        btnList[i]->show(screen);
    }
}

void MySDL::cleanup()
{
    for(int i = 0; i < NUMBER_OF_BUTTONS; i++)
    {
        clean_up(btnList[i]->getButtonImage());
    }
}

void MySDL::fillScreen()
{
    SDL_FillRect( screen, &screen->clip_rect, SDL_MapRGB( screen->format, 0xFF, 0xFF, 0xFF ) );
}

#pragma once
/*
int n_verts = 12;
float verts[] =
{
    //pos               col                     refl            nor
    -1.9f, -.7f, 1.f,    0.1f, 0.8f, 0.3f,         0.f,            0.f, 1.f, 0.f,         //tl
    -1.9f, -.7f, -3.f,    0.1f, 0.8f, 0.3f,        0.f,            0.f, 1.f, 0.f,              //tr
    1.9f, -.7f, -3.f,    0.1f, 0.8f, 0.3f,          0.f,            0.f, 1.f, 0.f,         //br

    -1.9f, -.7f, 1.f,    0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //tl
    1.9f, -.7f, -3.f,     0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //tr
    1.9f, -.7f, 1.f,     0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //br

    //pos               col                 
    .6f, -.7f, -3.f,    0.f, .1f, 0.f,         1.f,             -1.f, 0.f, 0.f,         //tl
    .6f, -.7f, 1.f,     0.f, .1f, 0.f,         1.f,            -1.f, 0.f, 0.f,            //tr
    .6f, .5f, 1.f,    0.f, .1f, 0.f,           1.f,             -1.f, 0.f, 0.f,           //br

    .6f, -.7f, -3.f,    0.f, .1f, 0.f,         1.f,             -1.f, 0.f, 0.f,           //tl
    .6f, .5f, 1.f,     0.f, .1f, 0.f,         1.f,             -1.f, 0.f, 0.f,          //tr
    .6f, .5f, -3.f,    0.f, .1f, 0.f,         1.f,              -1.f,0.f, 0.f,          //br

};
*/





int n_verts = 30;
float verts[] =
{
    //pos               col                     refl            nor
    -1.9f, -.7f, 1.f,    0.1f, 0.8f, 0.3f,         0.f,            0.f, 1.f, 0.f,         //tl
    -1.9f, -.7f, -3.f,    0.1f, 0.8f, 0.3f,        0.f,            0.f, 1.f, 0.f,              //tr
    1.9f, -.7f, -3.f,    0.1f, 0.8f, 0.3f,          0.f,            0.f, 1.f, 0.f,         //br

    -1.9f, -.7f, 1.f,    0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //tl
    1.9f, -.7f, -3.f,     0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //tr
    1.9f, -.7f, 1.f,     0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //br

    //pos               col                 
    .6f, -.7f, -3.f,    0.f, .1f, 0.f,         1.f,             -1.f, 0.f, 0.f,         //tl
    .6f, -.7f, 1.f,     0.f, .1f, 0.f,         1.f,            -1.f, 0.f, 0.f,            //tr
    .6f, .5f, 1.f,    0.f, .1f, 0.f,           1.f,             -1.f, 0.f, 0.f,           //br

    .6f, -.7f, -3.f,    0.f, .1f, 0.f,         1.f,             -1.f, 0.f, 0.f,           //tl
    .6f, .5f, 1.f,     0.f, .1f, 0.f,         1.f,             -1.f, 0.f, 0.f,          //tr
    .6f, .5f, -3.f,    0.f, .1f, 0.f,         1.f,              -1.f,0.f, 0.f,          //br

    //pos               col                 
    -.4f, .5f, -1.5f,    0.f, 0.f, 1.f,         0.f,            0.f, 0.f, 1.f,           //tl
    -.1f, .5f, -1.5f,     0.f, 0.f, 1.f,         0.f,           0.f, 0.f, 1.f,            //tr
    -.1f, -.5f, -1.5f,    0.f, 0.f, 1.f,         0.f,           0.f, 0.f, 1.f,            //br

    -.4f, .5f, -1.5f,    0.f, 0.f, .8f,         0.f,            0.f, 0.f, 1.f,           //tl
    -.1f, -.5f, -1.5f,    0.f, 0.f, .8f,         0.f,           0.f, 0.f, 1.f,            //br
    -.4f, -.5f, -1.5f,     0.f, 0.f, .8f,         0.f,          0.f, 0.f, 1.f,             //tr

    //pos               col                 
     .1f, .5f, -1.5f,    0.f, 0.f, 1.f,         0.f,            0.f, 0.f, 1.f,           //tl
    .4f, .5f, -1.5f,     0.f, 0.f, 1.f,         0.f,            0.f, 0.f, 1.f,           //tr
    .4f, -.5f, -1.5f,    0.f, 0.f, 1.f,         0.f,            0.f, 0.f, 1.f,           //br

    .1f, .5f, -1.5f,    0.f, 0.f, .8f,         0.f,             0.f, 0.f, 1.f,          //tl
    .4f, -.5f, -1.5f,    0.f, 0.f, .8f,         0.f,            0.f, 0.f, 1.f,           //br
    .1f, -.5f, -1.5f,     0.f, 0.f, .8f,         0.f,           0.f, 0.f, 1.f,            //tr

    //pos               col                 
    -.5f, -.5f, -1.5f,    .8f, 0.f, 0.f,         0.f,           1.f, 0.f, 0.f,            //tl
    -.5f, -.5f, -1.f,     .8f, 0.f, 0.f,         0.f,           1.f, 0.f, 0.f,            //tr
    -.5f, .5f, -1.f,     .8f, 0.f, 0.f,         0.f,            1.f, 0.f, 0.f,           //br

   -.5f, -.5f, -1.5f,    1.f, 0.f, 0.f,         0.f,            1.f, 0.f, 0.f,          //tl
    -.5f, .5f, -1.f,     1.f, .0f, 0.f,          0.f,           1.f, 0.f, 0.f,           //tr
    -.5f, .5f, -1.5f,    1.f, 0.f, 0.f,         0.f,            1.f, 0.f, 0.f           //br

};

/*
int n_verts = 12;
float verts[] =
{
    //pos               col                     refl            nor
    -.9f, -.7f, -1.f,    0.1f, 0.1f, 0.1f,         1.f,            0.f, 1.f, 0.f,         //tl
    -.9f, -.7f, -3.f,    0.1f, 0.1f, 0.1f,         1.f,            0.f, 1.f, 0.f,              //tr
    .9f, -.7f, -3.f,    0.1f, 0.1f, 0.1f,          1.f,            0.f, 1.f, 0.f,         //br

    -.9f, -.7f, -1.f,    0.1f, 0.1f, 0.1f,      1.f,            0.f, 1.f, 0.f,         //tl
    .9f, -.7f, -3.f,     0.1f, .10f, 0.1f,      1.f,            0.f, 1.f, 0.f,         //tr
    .9f, -.7f, -1.f,     0.1f, 0.1f, 0.1f,      1.f,            0.f, 1.f, 0.f,         //br

    //pos               col                 
    -.9f, .5f, -3.f,    0.f, 0.f, 1.f,         0.f,            0.f, 0.f, 1.f,           //tl
    -.1f, .5f, -3.f,     0.f, 0.f, 1.f,         0.f,           0.f, 0.f, 1.f,            //tr
    -.1f, -.7f, -3.f,    0.f, 0.f, 1.f,         0.f,           0.f, 0.f, 1.f,            //br

    -.9f, .5f, -3.f,    0.f, 0.f, .8f,         0.f,            0.f, 0.f, 1.f,           //tl
    -.1f, -.7f, -3.f,    0.f, 0.f, .8f,         0.f,           0.f, 0.f, 1.f,            //br
    -.9f, -.7f, -3.f,     0.f, 0.f, .8f,         0.f,          0.f, 0.f, 1.f,             //tr


};
*/
/*
int n_verts = 6;
float verts[] =
{

    //pos               col                     refl            nor
    -1.9f, -.7f, 1.f,    0.1f, 0.8f, 0.3f,         0.f,            0.f, 1.f, 0.f,         //tl
    -1.9f, -.7f, -3.f,    0.1f, 0.8f, 0.3f,        0.f,            0.f, 1.f, 0.f,              //tr
    1.9f, -.7f, -3.f,    0.1f, 0.8f, 0.3f,          0.f,            0.f, 1.f, 0.f,         //br

    -1.9f, -.7f, 1.f,    0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //tl
    1.9f, -.7f, -3.f,     0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //tr
    1.9f, -.7f, 1.f,     0.1f, 0.8f, 0.3f,      0.f,            0.f, 1.f, 0.f,         //br


};
*/
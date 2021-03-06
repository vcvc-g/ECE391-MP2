/* tab:4
 *
 * photo.c - photo display functions
 *
 * "Copyright (c) 2011 by Steven S. Lumetta."
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE AUTHOR OR THE UNIVERSITY OF ILLINOIS BE LIABLE TO
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHOR AND/OR THE UNIVERSITY OF ILLINOIS HAS BEEN ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE AUTHOR AND THE UNIVERSITY OF ILLINOIS SPECIFICALLY DISCLAIM ANY
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
 * PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND NEITHER THE AUTHOR NOR
 * THE UNIVERSITY OF ILLINOIS HAS ANY OBLIGATION TO PROVIDE MAINTENANCE,
 * SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Author:        Steve Lumetta
 * Version:       3
 * Creation Date: Fri Sep 9 21:44:10 2011
 * Filename:      photo.c
 * History:
 *    SL    1    Fri Sep 9 21:44:10 2011
 *        First written(based on mazegame code).
 *    SL    2    Sun Sep 11 14:57:59 2011
 *        Completed initial implementation of functions.
 *    SL    3    Wed Sep 14 21:49:44 2011
 *        Cleaned up code for distribution.
 */


#include <string.h>

#include "assert.h"
#include "modex.h"
#include "photo.h"
#include "photo_headers.h"
#include "world.h"

#define paletteOffset 64
#define layer2Size 64
#define layer4Size 4096
#define palette128 128
#define palette192 192
#define R 0
#define G 1
#define B 2


/* types local to this file(declared in types.h) */

/*
 * A room photo.  Note that you must write the code that selects the
 * optimized palette colors and fills in the pixel data using them as
 * well as the code that sets up the VGA to make use of these colors.
 * Pixel data are stored as one-byte values starting from the upper
 * left and traversing the top row before returning to the left of
 * the second row, and so forth.  No padding should be used.
 */
struct photo_t {
    photo_header_t hdr;            /* defines height and width */
    uint8_t        palette[192][3];     /* optimized palette colors */
    uint8_t*       img;                 /* pixel data               */
};

/*
 * An object image.  The code for managing these images has been given
 * to you.  The data are simply loaded from a file, where they have
 * been stored as 2:2:2-bit RGB values(one byte each), including
 * transparent pixels(value OBJ_CLR_TRANSP).  As with the room photos,
 * pixel data are stored as one-byte values starting from the upper
 * left and traversing the top row before returning to the left of the
 * second row, and so forth.  No padding is used.
 */
struct image_t {
    photo_header_t hdr;  /* defines height and width */
    uint8_t*       img;  /* pixel data               */
};


/*
 * A raw color from photo.
 */
struct color_t {
    int        r;             //red
    int        g;             //green
    int        b;             //blue
    int            c_count;   //color occurance
    int            o_idx;     //idx before sort
};



int l4_comp(const void *p, const void *q);    //comparator for sort



/* file-scope variables */

/*
 * The room currently shown on the screen.  This value is not known to
 * the mode X code, but is needed when filling buffers in callbacks from
 * that code(fill_horiz_buffer/fill_vert_buffer).  The value is set
 * by calling prep_room.
 */
static const room_t* cur_room = NULL;


/*
 * fill_horiz_buffer
 *   DESCRIPTION: Given the(x,y) map pixel coordinate of the leftmost
 *                pixel of a line to be drawn on the screen, this routine
 *                produces an image of the line.  Each pixel on the line
 *                is represented as a single byte in the image.
 *
 *                Note that this routine draws both the room photo and
 *                the objects in the room.
 *
 *   INPUTS:(x,y) -- leftmost pixel of line to be drawn
 *   OUTPUTS: buf -- buffer holding image data for the line
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
void fill_horiz_buffer(int x, int y, unsigned char buf[SCROLL_X_DIM]) {
    int            idx;   /* loop index over pixels in the line          */
    object_t*      obj;   /* loop index over objects in the current room */
    int            imgx;  /* loop index over pixels in object image      */
    int            yoff;  /* y offset into object image                  */
    uint8_t        pixel; /* pixel from object image                     */
    const photo_t* view;  /* room photo                                  */
    int32_t        obj_x; /* object x position                           */
    int32_t        obj_y; /* object y position                           */
    const image_t* img;   /* object image                                */

    /* Get pointer to current photo of current room. */
    view = room_photo(cur_room);

    /* Loop over pixels in line. */
    for (idx = 0; idx < SCROLL_X_DIM; idx++) {
        buf[idx] = (0 <= x + idx && view->hdr.width > x + idx ? view->img[view->hdr.width * y + x + idx] : 0);
    }

    /* Loop over objects in the current room. */
    for (obj = room_contents_iterate(cur_room); NULL != obj; obj = obj_next(obj)) {
        obj_x = obj_get_x(obj);
        obj_y = obj_get_y(obj);
        img = obj_image(obj);

        /* Is object outside of the line we're drawing? */
        if (y < obj_y || y >= obj_y + img->hdr.height || x + SCROLL_X_DIM <= obj_x || x >= obj_x + img->hdr.width) {
            continue;
        }

        /* The y offset of drawing is fixed. */
        yoff = (y - obj_y) * img->hdr.width;

        /*
         * The x offsets depend on whether the object starts to the left
         * or to the right of the starting point for the line being drawn.
         */
        if (x <= obj_x) {
            idx = obj_x - x;
            imgx = 0;
        }
        else {
            idx = 0;
            imgx = x - obj_x;
        }

        /* Copy the object's pixel data. */
        for (; SCROLL_X_DIM > idx && img->hdr.width > imgx; idx++, imgx++) {
            pixel = img->img[yoff + imgx];

            /* Don't copy transparent pixels. */
            if (OBJ_CLR_TRANSP != pixel) {
                buf[idx] = pixel;
            }
        }
    }
}


/*
 * fill_vert_buffer
 *   DESCRIPTION: Given the(x,y) map pixel coordinate of the top pixel of
 *                a vertical line to be drawn on the screen, this routine
 *                produces an image of the line.  Each pixel on the line
 *                is represented as a single byte in the image.
 *
 *                Note that this routine draws both the room photo and
 *                the objects in the room.
 *
 *   INPUTS:(x,y) -- top pixel of line to be drawn
 *   OUTPUTS: buf -- buffer holding image data for the line
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
void fill_vert_buffer(int x, int y, unsigned char buf[SCROLL_Y_DIM]) {
    int            idx;   /* loop index over pixels in the line          */
    object_t*      obj;   /* loop index over objects in the current room */
    int            imgy;  /* loop index over pixels in object image      */
    int            xoff;  /* x offset into object image                  */
    uint8_t        pixel; /* pixel from object image                     */
    const photo_t* view;  /* room photo                                  */
    int32_t        obj_x; /* object x position                           */
    int32_t        obj_y; /* object y position                           */
    const image_t* img;   /* object image                                */

    /* Get pointer to current photo of current room. */
    view = room_photo(cur_room);

    /* Loop over pixels in line. */
    for (idx = 0; idx < SCROLL_Y_DIM; idx++) {
        buf[idx] = (0 <= y + idx && view->hdr.height > y + idx ? view->img[view->hdr.width *(y + idx) + x] : 0);
    }

    /* Loop over objects in the current room. */
    for (obj = room_contents_iterate(cur_room); NULL != obj; obj = obj_next(obj)) {
        obj_x = obj_get_x(obj);
        obj_y = obj_get_y(obj);
        img = obj_image(obj);

        /* Is object outside of the line we're drawing? */
        if (x < obj_x || x >= obj_x + img->hdr.width ||
            y + SCROLL_Y_DIM <= obj_y || y >= obj_y + img->hdr.height) {
            continue;
        }

        /* The x offset of drawing is fixed. */
        xoff = x - obj_x;

        /*
         * The y offsets depend on whether the object starts below or
         * above the starting point for the line being drawn.
         */
        if (y <= obj_y) {
            idx = obj_y - y;
            imgy = 0;
        }
        else {
            idx = 0;
            imgy = y - obj_y;
        }

        /* Copy the object's pixel data. */
        for (; SCROLL_Y_DIM > idx && img->hdr.height > imgy; idx++, imgy++) {
            pixel = img->img[xoff + img->hdr.width * imgy];

            /* Don't copy transparent pixels. */
            if (OBJ_CLR_TRANSP != pixel) {
                buf[idx] = pixel;
            }
        }
    }
}


/*
 * image_height
 *   DESCRIPTION: Get height of object image in pixels.
 *   INPUTS: im -- object image pointer
 *   OUTPUTS: none
 *   RETURN VALUE: height of object image im in pixels
 *   SIDE EFFECTS: none
 */
uint32_t image_height(const image_t* im) {
    return im->hdr.height;
}


/*
 * image_width
 *   DESCRIPTION: Get width of object image in pixels.
 *   INPUTS: im -- object image pointer
 *   OUTPUTS: none
 *   RETURN VALUE: width of object image im in pixels
 *   SIDE EFFECTS: none
 */
uint32_t image_width(const image_t* im) {
    return im->hdr.width;
}

/*
 * photo_height
 *   DESCRIPTION: Get height of room photo in pixels.
 *   INPUTS: p -- room photo pointer
 *   OUTPUTS: none
 *   RETURN VALUE: height of room photo p in pixels
 *   SIDE EFFECTS: none
 */
uint32_t photo_height(const photo_t* p) {
    return p->hdr.height;
}


/*
 * photo_width
 *   DESCRIPTION: Get width of room photo in pixels.
 *   INPUTS: p -- room photo pointer
 *   OUTPUTS: none
 *   RETURN VALUE: width of room photo p in pixels
 *   SIDE EFFECTS: none
 */
uint32_t photo_width(const photo_t* p) {
    return p->hdr.width;
}


/*
 * prep_room
 *   DESCRIPTION: Prepare a new room for display.  You might want to set
 *                up the VGA palette registers according to the color
 *                palette that you chose for this room.
 *   INPUTS: r -- pointer to the new room
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: changes recorded cur_room for this file
 */
void prep_room(const room_t* r) {
    unsigned char *palette_RGB;
    /* Record the current room. */
    cur_room = r;
    //call function from modex.c
    photo_t *p = room_photo(r);
    palette_RGB = p->palette;
    fill_palette_octree(palette_RGB);
}


/*
 * read_obj_image
 *   DESCRIPTION: Read size and pixel data in 2:2:2 RGB format from a
 *                photo file and create an image structure from it.
 *   INPUTS: fname -- file name for input
 *   OUTPUTS: none
 *   RETURN VALUE: pointer to newly allocated photo on success, or NULL
 *                 on failure
 *   SIDE EFFECTS: dynamically allocates memory for the image
 */
image_t* read_obj_image(const char* fname) {
    FILE*    in;        /* input file               */
    image_t* img = NULL;    /* image structure          */
    uint16_t x;            /* index over image columns */
    uint16_t y;            /* index over image rows    */
    uint8_t  pixel;        /* one pixel from the file  */

    /*
     * Open the file, allocate the structure, read the header, do some
     * sanity checks on it, and allocate space to hold the image pixels.
     * If anything fails, clean up as necessary and return NULL.
     */
    if (NULL == (in = fopen(fname, "r+b")) ||
        NULL == (img = malloc(sizeof (*img))) ||
        NULL != (img->img = NULL) || /* false clause for initialization */
        1 != fread(&img->hdr, sizeof (img->hdr), 1, in) ||
        MAX_OBJECT_WIDTH < img->hdr.width ||
        MAX_OBJECT_HEIGHT < img->hdr.height ||
        NULL == (img->img = malloc
        (img->hdr.width * img->hdr.height * sizeof (img->img[0])))) {
        if (NULL != img) {
            if (NULL != img->img) {
                free(img->img);
            }
            free(img);
        }
        if (NULL != in) {
            (void)fclose(in);
        }
        return NULL;
    }

    /*
     * Loop over rows from bottom to top.  Note that the file is stored
     * in this order, whereas in memory we store the data in the reverse
     * order(top to bottom).
     */
    for (y = img->hdr.height; y-- > 0; ) {

        /* Loop over columns from left to right. */
        for (x = 0; img->hdr.width > x; x++) {

            /*
             * Try to read one 8-bit pixel.  On failure, clean up and
             * return NULL.
             */
            if (1 != fread(&pixel, sizeof (pixel), 1, in)) {
                free(img->img);
                free(img);
                (void)fclose(in);
                return NULL;
            }

            /* Store the pixel in the image data. */
            img->img[img->hdr.width * y + x] = pixel;
        }
    }

    /* All done.  Return success. */
    (void)fclose(in);
    return img;
}


/*
 * read_photo
 *   DESCRIPTION: Read size and pixel data in 5:6:5 RGB format from a
 *                photo file and create a photo structure from it.
 *                Code provided simply maps to 2:2:2 RGB.  You must
 *                replace this code with palette color selection, and
 *                must map the image pixels into the palette colors that
 *                you have defined.
 *   INPUTS: fname -- file name for input
 *   OUTPUTS: none
 *   RETURN VALUE: pointer to newly allocated photo on success, or NULL
 *                 on failure
 *   SIDE EFFECTS: dynamically allocates memory for the photo
 */
photo_t* read_photo(const char* fname) {
    FILE*    in;    /* input file               */
    photo_t* p = NULL;    /* photo structure          */
    uint16_t x;        /* index over image columns */
    uint16_t y;        /* index over image rows    */
    uint16_t pixel;    /* one pixel from the file  */

    struct color_t layer2[layer2Size];
    struct color_t layer4[layer4Size];
    uint16_t        cr, cg, cb, pixelR , pixelG ,pixelB, palR, palG, palB;
    int idx_2, idx_4, i_sort;
    int idx_i, i;      //loop index
    unsigned int decode_r, decode_g, decode_b;

    //layer4 initialization
    struct color_t p_color = {.r = 0, .g = 0, .b = 0, .c_count = 0, .o_idx = 0};
    for(idx_i = 0; idx_i < layer4Size; idx_i++){
        layer4[idx_i] = p_color;
    }
    //layer2 initialization
    for(idx_i = 0; idx_i < layer2Size; idx_i++){
        layer2[idx_i] = p_color;
    }

    /*
     * Open the file, allocate the structure, read the header, do some
     * sanity checks on it, and allocate space to hold the photo pixels.
     * If anything fails, clean up as necessary and return NULL.
     */
    if (NULL == (in = fopen(fname, "r+b")) ||
        NULL == (p = malloc(sizeof (*p))) ||
        NULL != (p->img = NULL) || /* false clause for initialization */
        1 != fread(&p->hdr, sizeof (p->hdr), 1, in) ||
        MAX_PHOTO_WIDTH < p->hdr.width ||
        MAX_PHOTO_HEIGHT < p->hdr.height ||
        NULL == (p->img = malloc
        (p->hdr.width * p->hdr.height * sizeof (p->img[0])))) {
        if (NULL != p) {
            if (NULL != p->img) {
                free(p->img);
            }
            free(p);
        }
        if (NULL != in) {
            (void)fclose(in);
        }
        return NULL;
    }

    /*
     * Loop over rows from bottom to top.  Note that the file is stored
     * in this order, whereas in memory we store the data in the reverse
     * order(top to bottom).
     */
    for (y = p->hdr.height; y-- > 0; ) {

        /* Loop over columns from left to right. */
        for (x = 0; p->hdr.width > x; x++) {

            /*
             * Try to read one 16-bit pixel.  On failure, clean up and
             * return NULL.
             */
            if (1 != fread(&pixel, sizeof (pixel), 1, in)) {
                free(p->img);
                free(p);
                (void)fclose(in);
                return NULL;
            }
            /*
             * 16-bit pixel is coded as 5:6:5 RGB(5 bits red, 6 bits green,
             * and 6 bits blue).  We change to 2:2:2, which we've set for the
             * game objects.  You need to use the other 192 palette colors
             * to specialize the appearance of each photo.
             *
             * In this code, you need to calculate the p->palette values,
             * which encode 6-bit RGB as arrays of three uint8_t's.  When
             * the game puts up a photo, you should then change the palette
             * to match the colors needed for that photo.
             */
            ////////////////////////////////////////////////////////////////////////////////////////////////
            //p->img[p->hdr.width * y + x] = (((pixel >> 14) << 4) | (((pixel >> 9) & 0x3) << 2) | ((pixel >> 3) & 0x3));
            /////////////////////////////////////////////////////////////////////////////////////////////////
            /*
            struct color_t {
                uint16_t        r, g, b;
                int            c_count;
                int            o_idx;
            };
            */


            ///EACH Pixel IN CURENT IMAGE
            // Current color idx in layer 4
            //RIGHT 12 THEN LEFT SHIFT 8 GET 4 MSB R AT IDX 8,9,10,11
            //RIGHT 3 MASK 0XF0  GET 4 MSB R AT IDX 4,5,6,7
            //RIGHT 1 MASK 0X0F GET 4 MSB R AT IDX 0,1,2,3
            //RRRRGGGGBBBB
            idx_4 = (((pixel >> 12) << 8) | ((pixel >> 3) & 0x0F0) | ((pixel >> 1) & 0x0F));


            cr = ((pixel >> 10) & 0x3E);    //RIGHT SHIFT 10 MASK 0X3E GET 6 bit R of current pixel
            cg = ((pixel >> 5) & 0x3F);    //RIGHT SHIFT 5 MASK 0X3F GET 6 bit G of current pixel
            cb = ((pixel << 1) & 0x3E);   //RIGHT SHIFT 1 MASK 0X3E GET 6 bit B of current pixel


            layer4[idx_4].r += cr;
            layer4[idx_4].g += cg;
            layer4[idx_4].b += cb;
            layer4[idx_4].c_count += 1;
            layer4[idx_4].o_idx = idx_4;

            //Current color idx in layer 2
            //RIGHT 14 THEN LEFT SHIFT 4 GET 2 MSB R AT IDX 4,5
            //RIGHT 9 MASK 0X03 THEN LEFT SHIFT 2 GET 2 MSB R AT IDX 2,3
            //RIGHT 3 THEN MASK 0X03 GET 2 MSB R AT IDX 0,1
            //RRGGBB
            idx_2 = (((pixel >> 14) << 4) | (((pixel >> 9) & 0x03) <<2 ) | ((pixel >> 3) & 0x03));
            layer2[idx_2].r += cr;
            layer2[idx_2].g += cg;
            layer2[idx_2].b += cb;
            layer2[idx_2].c_count += 1;
            layer2[idx_2].o_idx = 0;

        }
    }
    //OUT IMAGE Pixel

/*
    //move rgb sum to layer2
    for(i = 0; i < 4096; i++){
        idx_2 = i/64;
        layer2[idx_2].r += layer4[i].r;
        layer2[idx_2].g += layer4[i].g;
        layer2[idx_2].b += layer4[i].b;
        layer2[idx_2].c_count += layer4[i].c_count;

    }
*/
    //sort layer4 by color count
    qsort((void*)layer4 ,layer4Size,sizeof(struct color_t),l4_comp);

    //move rgb sum to layer2


    for(i = 0; i < palette128; i++){

        //mins first 128 layer4 rgb from layer2
        i_sort = layer4[i].o_idx;

        decode_r=(i_sort>>10)& 0x03; //RIGHT SHIFT 10 MASK WITH 0X03 GET 2 BIT R
		    decode_g=(i_sort>>6)& 0x03; //RIGHT SHIFT 6 MASK WITH 0X03 GET 2 BIT G
		    decode_b=(i_sort>>2)& 0x03; //RIGHT SHIFT 2 MASK WITH 0X03 GET 2 BIT B
		    i_sort=0;
        //INDED I_SORT IN FORMAT RRGGBB
		    i_sort=(decode_r<<4)+(decode_g<<2)+(decode_b);

        layer2[i_sort].r -= layer4[i].r;
        layer2[i_sort].g -= layer4[i].g;
        layer2[i_sort].b -= layer4[i].b;
        layer2[i_sort].c_count -= layer4[i].c_count;
        //avg first 128 layer4 rgb and put in palette

        if(layer4[i].c_count > 0){
            p->palette[i][R] = layer4[i].r/layer4[i].c_count;
            p->palette[i][G] = layer4[i].g/layer4[i].c_count;
            p->palette[i][B] = layer4[i].b/layer4[i].c_count;
        }
    }

    for(i = 0; i < layer2Size; i++){
        //avg first 128 layer4 rgb and put in palette
        if(layer2[i].c_count > 0){
            p->palette[palette128+i][R] = (layer2[i].r)/layer2[i].c_count;
            p->palette[palette128+i][G] = (layer2[i].g)/layer2[i].c_count;
            p->palette[palette128+i][B] = (layer2[i].b)/layer2[i].c_count;
        }
    }

    fseek(in, sizeof(p->hdr), SEEK_SET);

    for (y = p->hdr.height; y-- > 0; ) {
        /* Loop over columns from left to right. */
        for (x = 0; p->hdr.width > x; x++) {

            /*
             * Try to read one 16-bit pixel.  On failure, clean up and
             * return NULL.
             */
            if (1 != fread(&pixel, sizeof (pixel), 1, in)) {
                free(p->img);
                free(p);
                (void)fclose(in);
                return NULL;
            }


            pixelR = ((pixel >> 10) & 0x3E);    // RIGHT SHIFT 10 MASK WITH 0X3E 6 bit R of current pixel
            pixelG = ((pixel >> 5) & 0x3F);    // RIGHT SHIFT 5 MASK WITH 0X3F 6 bit G of current pixel
            pixelB = ((pixel << 1) & 0x3E);   // RIGHT SHIFT 1 MASK WITH 0X3E 6 bit b of current pixel

            for(i = palette128; i < palette192; i++){
                // 6 bit layer2 palette R/G/B value
                palR = p->palette[i][R];
                palG = p->palette[i][G];
                palB = p->palette[i][B];
                //compare front 2 MSB
                if((palR>>4==pixelR>>4)&&(palG>>4==pixelG>>4)&&(palB>>4==pixelB>>4)){
                    p->img[p->hdr.width * y + x] = i+paletteOffset;
                }
            }


            for(i = 0; i < palette128; i++){
                // 6 bit layer4 palette R/G/B value
                palR = p->palette[i][R];
                palG = p->palette[i][G];
                palB = p->palette[i][B];
                //compare front 4 MSB
                if((palR>>2==pixelR>>2)&&(palG>>2==pixelG>>2)&&(palB>>2==pixelB>>2)){
                    p->img[p->hdr.width * y + x] = i+paletteOffset;
                }
            }




        }
    }

    /* All done.  Return success. */
    (void)fclose(in);
    return p;
}


/*
 * l4_comp
 *   DESCRIPTION: COMPARATOR FOR SORT, DESENDING ORDER
 *   INPUTS: const void *p, const void *q
 *   RETURN VALUE: int buttons_pressed
 */
int l4_comp(const void *p, const void *q)
{
    int l = ((struct color_t *)p)->c_count;
    int r = ((struct color_t *)q)->c_count;
    return (r - l);
}

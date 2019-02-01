//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Sebastien Grange
    \version   3.2.0 $Rev: 2097 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileImageJPG.h"
//------------------------------------------------------------------------------
#ifdef C_USE_FILE_JPG
//------------------------------------------------------------------------------
#include <sstream>
#include <fstream>
#include <streambuf>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------
#include <setjmp.h>
extern "C" 
{
    #include "jpeglib.h"
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

// wrap any buffer into a stream
struct swrapper : public std::streambuf
{
    swrapper(char* s, std::size_t n)
    {
        setg(s, s, s + n);
    }
};

// Most of the following helper functions have been adapted from
// http://ai.stanford.edu/~acoates/jpegAndIOS.txt

const static size_t JPEG_BUF_SIZE = 16384;

struct my_error_mgr 
{
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buffer;
};

typedef struct my_error_mgr *my_error_ptr;

struct my_source_mgr
{
    struct jpeg_source_mgr pub;
    std::istream* is;
    JOCTET*       buffer;
};

METHODDEF (void) my_error_warn (j_common_ptr cinfo) 
{
    char buffer[JMSG_LENGTH_MAX];
    my_error_ptr myerr = (my_error_ptr) cinfo->err;
    myerr->pub.format_message (cinfo, buffer);
    printf ("jpeg error: %s\n", buffer);
    longjmp (myerr->setjmp_buffer, 1);
}

static void my_init_source(j_decompress_ptr cinfo) {}

static boolean my_fill_input_buffer(j_decompress_ptr cinfo)
{
    my_source_mgr* src = (my_source_mgr*)cinfo->src;
    src->is->read((char*)src->buffer, JPEG_BUF_SIZE);
    size_t bytes = (size_t)(src->is->gcount());
    if (bytes == 0)
    {
        src->buffer[0] = (JOCTET) 0xFF;
        src->buffer[1] = (JOCTET) JPEG_EOI;
        bytes = 2;
    }
    src->pub.next_input_byte = src->buffer;
    src->pub.bytes_in_buffer = bytes;
    return TRUE;
}

static void my_skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    my_source_mgr* src = (my_source_mgr*)cinfo->src;
    if (num_bytes > 0)
    {
        while (num_bytes > (long)src->pub.bytes_in_buffer)
        {
            num_bytes -= (long)src->pub.bytes_in_buffer;
            my_fill_input_buffer(cinfo);
        }
        src->pub.next_input_byte += num_bytes;
        src->pub.bytes_in_buffer -= num_bytes;
    }
}

static void my_term_source(j_decompress_ptr cinfo)
{
    my_source_mgr* src = (my_source_mgr*)cinfo->src;
    src->is->clear();
    src->is->seekg( src->is->tellg() - (std::streampos)src->pub.bytes_in_buffer );
}

static void my_set_source_mgr(j_decompress_ptr cinfo, std::istream& is)
{
    my_source_mgr* src = NULL;
    if (cinfo->src == 0)
    {
        cinfo->src = (struct jpeg_source_mgr *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(my_source_mgr));
        src = (my_source_mgr*) cinfo->src;
        src->buffer = (JOCTET *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, JPEG_BUF_SIZE*sizeof(JOCTET));
    }
    src->is = &is;
    src->pub.init_source = my_init_source;
    src->pub.fill_input_buffer = my_fill_input_buffer;
    src->pub.skip_input_data = my_skip_input_data;
    src->pub.resync_to_restart = jpeg_resync_to_restart;
    src->pub.term_source = my_term_source;
    src->pub.bytes_in_buffer = 0;
    src->pub.next_input_byte = 0;
}


//==============================================================================
/*!
    JPG decompressor, used by
    \ref bool cLoadJPG(cImage* a_image, void *a_buffer, int a_len) and
    \ref bool cLoadFileJPG(cImage* a_image, string a_filename).

    \param a_image  The source image to receive the decompressed JPG data.
    \param a_is     The input stream that contains the raw JPG data.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
static bool _loadJPG(cImage* a_image, std::istream &a_is)
{
    struct jpeg_decompress_struct cinfo;
    struct my_error_mgr           jerr;

    JSAMPARRAY  buffer;
    int         row_stride;

    // sanity check
    if (a_image == NULL)
        return false;

    // setup error routines
    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_warn;
    if (setjmp(jerr.setjmp_buffer))
    {
        jpeg_destroy_decompress(&cinfo);
        return (C_ERROR);
    }

    // allocate jpeg decompressor
    jpeg_create_decompress(&cinfo);

    // specify data source (the stream)
    my_set_source_mgr(&cinfo, a_is);

    // read file parameters with jpeg_read_header()
    jpeg_read_header(&cinfo, TRUE);

    // start decompressor
    jpeg_start_decompress(&cinfo);
    int width  = (int)(cinfo.output_width);
    int height = (int)(cinfo.output_height);

    // we allocate memory for image
    if ((cinfo.out_color_components == 1 && !a_image->allocate(width, height, GL_LUMINANCE)) ||
        (cinfo.out_color_components == 3 && !a_image->allocate(width, height, GL_RGB)))
    {
        return (C_ERROR);
    }

    // retrieve pointer to image data
    unsigned char* data = a_image->getData();

    // allocate temporary buffer
    row_stride = cinfo.output_width * cinfo.output_components;
    buffer =(*cinfo.mem->alloc_sarray)((j_common_ptr) & cinfo, JPOOL_IMAGE, row_stride, 1);

    // while (scan lines remain to be read)
    int line = 0;
    while (cinfo.output_scanline < cinfo.output_height)
    {
        jpeg_read_scanlines(&cinfo, buffer, 1);
        int index = 3*width*(height-1-line++);
        unsigned char *ptr = buffer[0];
        if (cinfo.out_color_components == 3)
        {
            for (int count=0; count<width; count++)
            {
                for (int channel=0; channel<cinfo.out_color_components; channel++)
                {
                    data[index++] = *ptr++;
                }
            }
        }
    }

    // finish decompression
    jpeg_finish_decompress(&cinfo);

    // release JPEG decompression object
    jpeg_destroy_decompress(&cinfo);

    // return success
    return (C_SUCCESS);
}

//==============================================================================
//
// Most of the following helper functions have been adapted from
// http://ai.stanford.edu/~acoates/jpegAndIOS.txt
//
//==============================================================================

struct my_destination_mgr
{
  struct jpeg_destination_mgr pub;
  ostream* os;
  JOCTET * buffer;
};

static void my_init_destination (j_compress_ptr cinfo)
{
    my_destination_mgr* dest = (my_destination_mgr*) cinfo->dest;
    dest->buffer = (JOCTET *)(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE, JPEG_BUF_SIZE * sizeof(JOCTET));
    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = JPEG_BUF_SIZE;
}

static boolean my_empty_output_buffer(j_compress_ptr cinfo)
{
    my_destination_mgr* dest = (my_destination_mgr*)cinfo->dest;
    dest->os->write((const char*)dest->buffer, JPEG_BUF_SIZE);
    if (dest->os->fail())
        printf("JPEG: couldn't write entire jpeg buffer to stream.\n");
    dest->pub.next_output_byte = dest->buffer;
    dest->pub.free_in_buffer = JPEG_BUF_SIZE;
    return TRUE;
}

static void my_term_destination (j_compress_ptr cinfo)
{
    my_destination_mgr* dest = (my_destination_mgr*) cinfo->dest;
    size_t datacount = JPEG_BUF_SIZE - dest->pub.free_in_buffer;
    if (datacount > 0)
    {
        dest->os->write((const char*)dest->buffer, datacount);
        if (dest->os->fail())
            printf("JPEG: couldn't write remaining jpeg data to stream.\n");
    }
    dest->os->flush();
}

static void my_set_dest_mgr(j_compress_ptr cinfo, std::ostream& os)
{
    my_destination_mgr* dest;
    if (cinfo->dest == NULL)
    {
        cinfo->dest = (struct jpeg_destination_mgr *)
            (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(my_destination_mgr));
    }
    dest = (my_destination_mgr*)cinfo->dest;
    dest->pub.init_destination = my_init_destination;
    dest->pub.empty_output_buffer = my_empty_output_buffer;
    dest->pub.term_destination = my_term_destination;
    dest->os = &os;
}


//==============================================================================
/*!
    JPG compressor, used by
    \ref bool cSaveJPG(cImage* a_image, char **a_buffer, int *a_len) and
    \ref bool cSaveFileJPG(cImage* a_image, string a_filename)

    \param a_image  The source image to compress to JPG data.
    \param a_os     The output stream to write the compressed JPG data to.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
static bool _saveJPG(cImage* a_image, std::ostream &a_os)
{
    const int quality = 80;

    struct jpeg_compress_struct  cinfo;
    struct my_error_mgr          jerr;

    // sanity check
    if (a_image == NULL)
        return (C_ERROR);

    // retrieve image size
    unsigned int width  = a_image->getWidth();
    unsigned int height = a_image->getHeight();
    if (!((width > 0) && (height > 0)))
        return (C_ERROR);

    // retrieve pointer to data
    unsigned char* data = a_image->getData();

    // allocate and initialize JPEG compression object
    cinfo.err           = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_warn;
    if (setjmp(jerr.setjmp_buffer))
    {
        jpeg_destroy_compress(&cinfo);
        return false;
    }
    jpeg_create_compress(&cinfo);

    // assign output
    my_set_dest_mgr(&cinfo, a_os);

    // set parameters for compression
    cinfo.image_width      = width;
    cinfo.image_height     = height;
    switch (a_image->getFormat())
    {
    case GL_RGB:
        cinfo.input_components = 3;
        cinfo.in_color_space   = JCS_RGB;
        break;

    case GL_LUMINANCE:
        cinfo.input_components = 1;
        cinfo.in_color_space   = JCS_GRAYSCALE;
        break;

        // unsupported
    case GL_LUMINANCE_ALPHA:
    case GL_RGBA:
    default:
        jpeg_destroy_compress(&cinfo);
        return (C_ERROR);
    }

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    // start compressor
    jpeg_start_compress(&cinfo, TRUE);

    // write all scanlines at once
    for (int j=height; j>0; j--)
    {
        unsigned char *ptr = data + cinfo.input_components*width*(j-1);
        jpeg_write_scanlines(&cinfo, &ptr, 1);
    }

    // finish compression
    jpeg_finish_compress(&cinfo);

    // release JPEG compression object
    jpeg_destroy_compress(&cinfo);

    // return success
    return (C_SUCCESS);
}

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function loads a JPG image from a file into a cImage structure. \n
    If the operation succeeds, then the functions returns __true__ and the 
    image data is loaded into image structure a_image.n
    If the operation fails, then the function returns __false__.\n
    In both cases, previous image information stored in a_image is erased.

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileJPG(cImage* a_image, const std::string& a_filename)
{
    ifstream in;
    in.open(a_filename.c_str(), ios::in | ios::binary);
    if (!in.is_open())
    {
        return (C_ERROR);
    }

    return (_loadJPG(a_image, in));
}


//==============================================================================
/*!
    This function loads a JPG image from a memory buffer into a cImage structure. \n
    If the operation succeeds, then the functions returns __true__ and the
    image data is loaded into image structure a_image. \n
    If the operation fails, then the function returns __false__. \n
    In both cases, previous image information stored in a_image is erased.

    \param  a_image   Image structure.
    \param  a_buffer  Memory buffer containing JPG data.
    \param  a_len     Buffer size in bytes.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadJPG(cImage* a_image, const unsigned char *a_buffer, unsigned int a_len)
{
    swrapper buf((char*)a_buffer, a_len);
    istream in(&buf);

    return (_loadJPG (a_image, in));
}


//==============================================================================
/*!
    This function saves a JPG image from a cImage structure to a file. \n
    If the operation succeeds, then the functions returns __true__ and the
    image data is saved to a file. \n
    If the operation fails, then the function returns __false__.

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFileJPG(cImage* a_image, const std::string& a_filename)
{
    ofstream out;
    out.open(a_filename.c_str(), ios::out | ios::binary);

    return (_saveJPG(a_image, out));
}


//==============================================================================
/*!
    This function saves a JPG image from a cImage structure to a memory buffer. \n
    If the operation succeeds, then the functions returns __true__ and the
    image data is saved to a file. \n
    If the operation fails, then the function returns __false__.

    \param  a_image   Image structure.
    \param  a_buffer  Returned memory buffer where JPG data is stored.
    \param  a_len     Returned buffer size in bytes.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveJPG(cImage* a_image, unsigned char **a_buffer, unsigned int *a_len)
{
    ostringstream out;
    bool result = _saveJPG(a_image, out);

    if (result)
    {
        *a_len    = (unsigned int)(out.str().size());
        *a_buffer = (unsigned char*)malloc(*a_len);
        memcpy(*a_buffer, out.str().c_str(), out.str().size());
    }

    return (result);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif // C_USE_FILE_JPG
//------------------------------------------------------------------------------

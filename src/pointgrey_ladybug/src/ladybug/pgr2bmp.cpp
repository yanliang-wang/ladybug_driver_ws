//=============================================================================
//
// pgr2bmp.cpp
// 2020.9.19
//
// This program is used to extract bmp images from a Ladybug source stream(.pgr)
// The format of output images' name is "camera0/1/2/3/4/5-imageid.bmp"
//
//=============================================================================
//
// example:
//    pgr2bmp gr2bmp pgrFileName outputDirectory
//
//=============================================================================


#include <cstdio>
#include <string>
#include <iostream>
#include <ladybugstream.h>

#define _MAX_PATH 256

#define _HANDLE_ERROR \
    if( error != LADYBUG_OK ) \
{ \
    printf( "Error! Ladybug library reported %s\n", \
    ::ladybugErrorToString( error ) ); \
    goto _EXIT; \
}

using namespace std;

//
// echo program usage
//
void usage()
{
    printf (
            "Usage :\n"
            "\t pgr2bmp pgrFileName outputDirectory \n"
    );
}


int main(int argc, char* argv[])
{

    // At least two parameters are needed
    if (argc != 3)
    {
        usage();
        return 0;
    }
    //
    // save parameters
    //
    char* gprFileName  = argv[1];
    char fileNmae[256] = { 0 };
    const std::string outputPathPrefix = std::string (argv[2]);
    std::string outputPath;
    LadybugColorProcessingMethod colorProcessingMethod = LADYBUG_HQLINEAR;

    //
    // Create stream context for reading
    //
    LadybugStreamContext readingContext;
    LadybugError error = ladybugCreateStreamContext( &readingContext );
    _HANDLE_ERROR

    //
    // Open the source stream file
    //
    printf( "Opening source stream file : %s\n", gprFileName);
    error = ladybugInitializeStreamForReading( readingContext, gprFileName, true );
    _HANDLE_ERROR

    //
    // Create context
    //
    LadybugContext context;
    error = ladybugCreateContext( &context);
    _HANDLE_ERROR

    //
    // Read the stream header
    //
    LadybugStreamHeadInfo streamHeaderInfo;
    error = ladybugGetStreamHeader( readingContext, &streamHeaderInfo );
    _HANDLE_ERROR

    //
    // Set color processing method.
    //
    printf("Setting debayering method...\n" );
    error = ladybugSetColorProcessingMethod( context, colorProcessingMethod);
    _HANDLE_ERROR

    //
    // convert and save the image
    //
    // Seek the position of the first image
    error = ladybugGoToImage( readingContext, 0 );
    _HANDLE_ERROR
    {
        // Get the total number of the images in these stream files
        unsigned int uiNumOfImages = 0;
        error = ladybugGetStreamNumOfImages( readingContext, &uiNumOfImages );
        _HANDLE_ERROR
        printf( "The source stream file has %u images.\n", uiNumOfImages );

        // Copy all the specified images to the destination file
        for (unsigned int currIndex = 0; currIndex <= uiNumOfImages; currIndex++ )
        {
            printf( "Copying %u of %u\n", currIndex+1,  uiNumOfImages ) ;

            // Read a Ladybug image from stream
            LadybugImage currentImage;
            error = ladybugReadImageFromStream( readingContext, &currentImage );
            _HANDLE_ERROR

            // Allocate memory for the 6 processed images
            unsigned char* arpBuffers[LADYBUG_NUM_CAMERAS] = {nullptr };
            for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++)
            {
                arpBuffers[uiCamera] = new unsigned char[currentImage.uiRows * currentImage.uiCols * 4];
            }

            // Convert the image to BGRU format texture buffers
            printf("Converting image...\n");
            error = ::ladybugConvertImage(context, &currentImage, arpBuffers, LADYBUG_BGRU);
            _HANDLE_ERROR

            // Save the image as 6 individual raw (unstitched, distorted) images
            printf("Saving images...\n");
            for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++)
            {
                LadybugProcessedImage processedImage;
                processedImage.pData = arpBuffers[uiCamera];
                processedImage.pixelFormat = LADYBUG_BGRU;
                processedImage.uiCols = currentImage.uiCols;
                processedImage.uiRows = currentImage.uiRows;
                // save to bmp
                sprintf(fileNmae, "/camera%d-%d.bmp", uiCamera,currentImage.imageInfo.ulSequenceId);
                outputPath = outputPathPrefix + string(fileNmae) ;
                cout << outputPath << endl;
                error = ladybugSaveImage(readingContext, &processedImage, outputPath.c_str(), LADYBUG_FILEFORMAT_BMP);
                _HANDLE_ERROR
                printf("Saved camera %u image to %s.\n", uiCamera, outputPath.c_str());
            }
        }
    }

    _EXIT:
    // Close the reading and writing stream
    error = ladybugStopStream( readingContext );

    // Destroy stream context
    ladybugDestroyStreamContext ( &readingContext );
    return 0;
}

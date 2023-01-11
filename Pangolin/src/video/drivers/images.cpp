/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <pangolin/video/drivers/images.h>
#include <pangolin/utils/file_utils.h>

#include <cstring>

namespace pangolin
{

bool ImagesVideo::QueueFrame()
{
    if(num_loaded < num_files) {
        Frame frame;
        for(size_t c=0; c< num_channels; ++c) {
            const std::string& filename = Filename(num_loaded,c);
            frame.push_back( LoadImage( filename ) );
        }
        loaded.push_back(frame);
        ++num_loaded;
        return true;
    }
    return false;
}

ImagesVideo::ImagesVideo(const std::string& wildcard_path)
    : num_files(-1), num_channels(0),
      num_loaded(0)
{
    const std::vector<std::string> wildcards = Expand(wildcard_path, '[', ']', ',');
    num_channels = wildcards.size();
    
    filenames.resize(num_channels);
    
    for(size_t i = 0; i < wildcards.size(); ++i) {
        const std::string channel_wildcard = PathExpand(wildcards[i]);
        FilesMatchingWildcard(channel_wildcard, filenames[i]);
        if(num_files < 0) {
            num_files = (int)filenames[i].size();
        }else{
            if( num_files != (int)filenames[i].size() ) {
                std::cerr << "Warning: Video Channels have unequal number of files" << std::endl;
            }
            num_files = std::min(num_files, (int)filenames[i].size());
        }
        if(num_files == 0) {
            throw VideoException("No files found for wildcard '" + channel_wildcard + "'");
        }
    }
    
    // Load first image in order to determine stream sizes etc
    QueueFrame();
    
    size_bytes = 0;
    for(size_t c=0; c < num_channels; ++c) {
        const TypedImage& img = loaded[0][c];
        const StreamInfo stream_info(img.fmt, img.w, img.h, img.pitch, (unsigned char*)0 + size_bytes);
        streams.push_back(stream_info);        
        size_bytes += img.h*img.pitch;
    }
    
    // TODO: Queue frames in another thread.
}

ImagesVideo::~ImagesVideo()
{
    
}

//! Implement VideoInput::Start()
void ImagesVideo::Start()
{
    
}

//! Implement VideoInput::Stop()
void ImagesVideo::Stop()
{
    
}

//! Implement VideoInput::SizeBytes()
size_t ImagesVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& ImagesVideo::Streams() const
{
    return streams;
}

//! Implement VideoInput::GrabNext()
bool ImagesVideo::GrabNext( unsigned char* image, bool wait )
{
    QueueFrame();
        
    if(!loaded.size()) return false;
    
    Frame frame = loaded.front();
    loaded.pop_front();
            
    for(size_t c=0; c < num_channels; ++c){
        TypedImage& img = frame[c];
        if(!img.ptr) return false;
        const StreamInfo& si = streams[c];
        std::memcpy(image + (size_t)si.Offset(), img.ptr, si.SizeBytes());
        img.Dealloc();
    }
    return true;
}

//! Implement VideoInput::GrabNewest()
bool ImagesVideo::GrabNewest( unsigned char* image, bool wait )
{
    return GrabNext(image,wait);
}

}

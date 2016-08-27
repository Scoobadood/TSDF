#ifndef PNGWRAPPER_H
#define PNGWRAPPER_H

#include <string>

class PngWrapper {
public:
    enum PNG_TYPE {
        GREYSCALE_8,
        GREYSCALE_16,
        COLOUR
    };


    PngWrapper( const std::string& file_name, PNG_TYPE type = GREYSCALE_16 );
    PngWrapper( const uint16_t width, const uint16_t height, const uint8_t * data, PNG_TYPE );
    virtual ~PngWrapper();

    uint32_t width();
    uint32_t height();
    bool save_to( const std::string& file_name ) const;

protected:
private:
    uint32_t        m_width;
    uint32_t        m_height;

    const uint8_t  *m_data;

    PNG_TYPE        m_type;

};

#endif // PNGWRAPPER_H

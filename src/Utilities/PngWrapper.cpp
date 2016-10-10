#include "../include/PngWrapper.hpp"
#include "../include/PngUtilities.hpp"

PngWrapper::PngWrapper(const std::string& file_name, PNG_TYPE type ) {
    m_type = type;
    switch(type ) {
        case COLOUR:
            m_data = reinterpret_cast<uint8_t *> (load_colour_png_from_file(file_name, m_width, m_height));
            break;

        case GREYSCALE_16:
            m_data = reinterpret_cast<uint8_t *> (load_png_from_file(file_name, m_width, m_height));
            break;
            
        case GREYSCALE_8:
            break;
    }

    if( !m_data) {
         throw std::invalid_argument( "Failed to create PNGWrapper" );
    }
}

PngWrapper::PngWrapper( const uint16_t width, const uint16_t height, const uint8_t * data, PNG_TYPE type ) {
    m_width = width;
    m_height = height;
    uint64_t sz = width * height;

    switch( type ) {
    case GREYSCALE_8:
        break;

    case GREYSCALE_16:
        sz *= 2;
        break;

    case COLOUR:
        sz *= 3;
        break;
    }
    m_data = new uint8_t[sz];
    if( m_data) {
        memcpy( (void *)m_data, data, sz );
    } else {
        throw new std::bad_alloc();
    }
    m_type = type;
}


PngWrapper::~PngWrapper() {
    if( m_data ) {
        delete [] m_data;
        m_data = 0;
    }
    m_width = 0;
    m_height = 0;
}

bool PngWrapper::save_to( const std::string& file_name ) const {
    bool saved_ok = false;
    switch( m_type ) {
    case COLOUR:
        saved_ok = save_colour_png_to_file(file_name, m_width, m_height, m_data);
        break;

    case GREYSCALE_8:
        saved_ok = save_png_to_file( file_name, m_width, m_height, m_data);
        break;

    case GREYSCALE_16:
        saved_ok = save_png_to_file( file_name, m_width, m_height, (uint16_t *) m_data);
        break;
    }
    return saved_ok;
}

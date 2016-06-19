#include "PngWrapper.hpp"
#include "PngUtilities.hpp"

PngWrapper::PngWrapper(const std::string& file_name ) {
    m_data = load_png_from_file(file_name, m_width, m_height);
    m_type = GREYSCALE;
}

PngWrapper::PngWrapper( const uint16_t width, const uint16_t height, const uint8_t * data, PNG_TYPE ) {
    m_width = width;
    m_height = height;
    m_data = reinterpret_cast<const uint16_t *>(data);
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
    if( m_type == COLOUR ) {
        saved_ok = save_colour_png_to_file(file_name, m_width, m_height, (uint8_t *) m_data);
    } else {
        saved_ok = save_png_to_file( file_name, m_width, m_height, (uint16_t *) m_data);
    }
}

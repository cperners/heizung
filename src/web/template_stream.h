#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct HeatingTemplateValue { const char* name; const char* value; };

// ESP32 flash is memory mapped. Only explicitly listed tokens are replaced;
// percent signs in CSS, JavaScript or replacement values remain literal.
class HeatingTemplateStream {
  const char* source;
  const size_t sourceLength;
  const HeatingTemplateValue* values;
  const size_t valueCount;
  size_t position = 0;
  const char* replacement = nullptr;
  size_t replacementPosition = 0;
public:
  HeatingTemplateStream(const char* html, const HeatingTemplateValue* tokens, size_t count)
    : source(html), sourceLength(strlen(html)), values(tokens), valueCount(count) {}

  size_t read(uint8_t* output, size_t capacity) {
    size_t written = 0;
    while (written < capacity) {
      if (replacement) {
        if (replacement[replacementPosition]) {
          output[written++] = static_cast<uint8_t>(replacement[replacementPosition++]);
          continue;
        }
        replacement = nullptr;
      }
      if (position >= sourceLength) break;
      if (source[position] == '%') {
        for (size_t i=0; i<valueCount; ++i) {
          const size_t length = strlen(values[i].name);
          if (sourceLength-position >= length+2 &&
              memcmp(source+position+1, values[i].name, length) == 0 &&
              source[position+length+1] == '%') {
            position += length+2;
            replacement = values[i].value;
            replacementPosition = 0;
            break;
          }
        }
        if (replacement) continue;
      }
      output[written++] = static_cast<uint8_t>(source[position++]);
    }
    return written;
  }

  size_t renderedLength() const {
    HeatingTemplateStream copy = *this;
    uint8_t buffer[128];
    size_t result = 0, count;
    while ((count = copy.read(buffer, sizeof(buffer))) != 0) result += count;
    return result;
  }
};

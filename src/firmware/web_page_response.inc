#include <WebResponseImpl.h>
#include "web/template_stream.h"

static const char* const pagePlaceholderNames[] = {
  "TimeString",
  "MQTTUPDATE",
  "HEIZUNGSPUMPE",
  "BOILERPUMPE",
  "TROOM",
  "TAUSSEN",
  "TKESSEL",
  "TVORLAUF",
  "TBOILER",
  "BOILERINFO",
  "BOILERDELTA",
  "BOILERSOLL",
  "BOILERDIFF",
  "VORLAUFMAX",
  "KURVENSTEILHEIT",
  "RESTWAERMEZIEL",
  "TKACHELOFEN",
  "KACHELOFENSTATUS",
  "REGELUNGSART",
  "KACHELOFENEIN",
  "KACHELOFENAUS",
  "WIFISSID",
  "WIFIRSSI",
  "BRENNERSPERRE",
  "WEBTIME",
  "MODEAUTO",
  "MODERAUM",
  "MODEAUSSEN",
  "GASDAY",
  "GASTOTAL",
  "BETRIEBSART",
  "BETRIEBAUTO",
  "BETRIEBBOILER",
  "BETRIEBHEIZUNG",
  "BETRIEBAUS",
  "RAUMTAG",
  "RAUMNACHT",
  "RAUMURLAUB",
  "BETRIEBURLAUB",
  "TAGZEIT",
  "NACHTZEIT"
};
constexpr size_t PAGE_PLACEHOLDER_COUNT = sizeof(pagePlaceholderNames)/sizeof(pagePlaceholderNames[0]);

class HeatingPageResponse final : public AsyncAbstractResponse {
  String values[PAGE_PLACEHOLDER_COUNT];
  HeatingTemplateValue replacements[PAGE_PLACEHOLDER_COUNT];
  HeatingTemplateStream stream;
public:
  HeatingPageResponse() : AsyncAbstractResponse(nullptr), stream(index_html, replacements, PAGE_PLACEHOLDER_COUNT) {
    for (size_t i=0; i<PAGE_PLACEHOLDER_COUNT; ++i) {
      values[i] = processor(String(pagePlaceholderNames[i]));
      replacements[i] = {pagePlaceholderNames[i], values[i].c_str()};
    }
    _code = 200;
    _contentType = "text/html; charset=utf-8";
    _contentLength = stream.renderedLength();
    addHeader("Cache-Control", "no-store");
  }
  bool _sourceValid() const override { return true; }
  size_t _fillBuffer(uint8_t* data, size_t length) override {
    return stream.read(data, length);
  }
};

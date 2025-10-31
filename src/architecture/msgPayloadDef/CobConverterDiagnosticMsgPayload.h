#ifndef COB_CONVERTER_DIAGNOSTIC_MESSAGE_H
#define COB_CONVERTER_DIAGNOSTIC_MESSAGE_H

typedef struct {
    bool coberrorOutlierTrigger;  // true if the predicted COB error >= numStandardDeviations * Standard deviations
} CobConverterDiagnosticMsgPayload;

#endif

#ifndef CELESTIAL_BODY_PARAMETERS
#define CELESTIAL_BODY_PARAMETERS

#define MAX_STRING_LENGTH 256
#define MAX_PARAMETER_LENGTH 12

/*! @brief Celestial body parameters message*/

typedef struct {
    char bodyName[MAX_STRING_LENGTH];
    char shapeModel[MAX_STRING_LENGTH];
    double sigma_BN[3];
    int perlinNoiseOctaveCount;
    double perlinNoiseBaseFrequency;
    double perlinNoiseBaseAmplitude;
    double perlinNoisePersistence;
    double proceduralRocks;
    char brdf[MAX_STRING_LENGTH];
    double reflectanceParameters[MAX_PARAMETER_LENGTH];
    double meanRadius;
    double principalAxisDistortion[3];
    double isotropicScattering;
    double geometricAlbedo;
} CelestialBodyParametersMsgPayload;

#endif  // CELESTIAL_BODY_PARAMETERS

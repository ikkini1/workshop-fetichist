#include "ComputerCard.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"

#include "tusb.h"
#include "usb_midi_host.h"

#include <atomic>
#include <cmath>
#include <cstdint>

void (*ComputerCard::audio_callback_ptr)(void*) = nullptr;
void *ComputerCard::audio_callback_inst = nullptr;

// ============================================================
// CONFIGURATION
// ============================================================

static constexpr uint32_t SAMPLE_RATE = 24000;
static constexpr uint32_t TRIGGER_LENGTH_SAMPLES = SAMPLE_RATE / 200; // 5 ms
static constexpr uint32_t MIDI_CLOCK_TIMEOUT_US = 500000;             // 500 ms

// ============================================================
// MIDI : données partagées entre les deux cœurs
// ============================================================

static std::atomic<uint32_t> midiClockTicks{0};
static std::atomic<uint32_t> lastMidiClockUs{0};
static std::atomic<int> midiBpmReceived{0};
static std::atomic<bool> midiClockEverSeen{false};
static std::atomic<bool> midiTransportStopped{false};
static std::atomic<bool> midiResetRequested{false};

// ============================================================
// SÉQUENCEUR
// ============================================================

class TuringSequencer : public ComputerCard
{
public:
    USBPowerState_t GetUSBPowerState() { return USBPowerState(); }
    void Initialise()
    {
        EnableNormalisationProbe();

        effectsDryWet = 0.0f;

        PulseOut1(false);
        PulseOut2(false);

        CVOut1Millivolts(0);
        CVOut2Millivolts(0);

        randomState = static_cast<uint32_t>(time_us_64())
                    ^ static_cast<uint32_t>(UniqueCardID());

        if (randomState == 0)
            randomState = 0xA341316C;
    }

    void ProcessSample() override
    {
        ++sampleCounter;

// ----------------------------------------------------
// Mixer audio : Audio 1 IN + Audio 2 IN
// ----------------------------------------------------

const int32_t audio1 = (AudioIn1() * 70) / 100;
const int32_t audio2 = (AudioIn2() * 70) / 100;

int32_t mixerOutput = audio1 + audio2;

// Protection contre le dépassement de la plage de sortie.
if (mixerOutput > 2047)
    mixerOutput = 2047;

if (mixerOutput < -2048)
    mixerOutput = -2048;

// ----------------------------------------------------
// Générateur de bruit blanc
// ----------------------------------------------------

const int16_t noiseSample = GenerateWhiteNoise();

// Le bruit reste disponible indépendamment sur Audio 2 OUT.
AudioOut2(noiseSample);

// ----------------------------------------------------
// Crossfader : mixer vers bruit blanc
// ----------------------------------------------------

// CV2 externe prioritaire.
// Sinon, CV1 du séquenceur atténué par X.
int32_t crossfadeCV;

if (Connected(CV2))
{
    crossfadeCV = CVIn2();
}
else
{
    // X du séquenceur reste mémorisé en mode Down,
    // mais les nouvelles valeurs de CV continuent d'agir.
    crossfadeCV =
        (sequencerCV * sequencerXValue) / 4095;
}

// Limitation à la plage positive 0-5V.
if (crossfadeCV < 0) crossfadeCV = 0;
if (crossfadeCV > 2047) crossfadeCV = 2047;

// Conversion en coefficient de mélange entre 0 et 1.
const float crossfadePosition =
    static_cast<float>(crossfadeCV) / 2047.0f;

// Mélange progressif des deux sources.
const float mixedOutput =
    static_cast<float>(mixerOutput) * (1.0f - crossfadePosition)
    + static_cast<float>(noiseSample) * crossfadePosition;

// ----------------------------------------------------
// Effets Rust / Corrosion
// ----------------------------------------------------

// ----------------------------------------------------
// Réglages des effets : actifs uniquement avec Z DOWN
// ----------------------------------------------------



const float dryWet = effectsDryWet;
const float effectMix = rustCorrosionMix;
// Signal original.
const float drySignal = mixedOutput;

// ----------------------------------------------------
// Traitement parallèle Rust / Corrosion
// ----------------------------------------------------

// Intensité interne fixe de chaque effet.
const float rustSignal = ProcessRust(mixedOutput, 1.0f) * 0.708f;
const float corrosionSignal = ProcessCorrosion(mixedOutput, 0.55f) * 0.708f;

// X dose le mélange entre Rust et Corrosion.
// À gauche : Rust. À droite : Corrosion.
const float processedSignal =
    rustSignal * (1.0f - effectMix)
    + corrosionSignal * effectMix;

// Mélange entre signal original et signal traité.
const float effectOutput =
    drySignal * (1.0f - dryWet)
    + processedSignal * dryWet;

// Sortie principale.
int32_t finalOutput = static_cast<int32_t>(effectOutput);

// ----------------------------------------------------
// VCA : contrôle du volume par CV1 IN
// ----------------------------------------------------

int32_t vcaLevel = 2047;

// Si CV1 IN est connecté, il contrôle le niveau.
// Sinon, le VCA reste complètement ouvert.

if (Connected(CV1))
{
    vcaLevel = CVIn1();

    if (vcaLevel < 0)
        vcaLevel = 0;

    if (vcaLevel > 2047)
        vcaLevel = 2047;
}

finalOutput = (finalOutput * vcaLevel) / 2047;

if (finalOutput > 2047)
    finalOutput = 2047;

if (finalOutput < -2048)
    finalOutput = -2048;

AudioOut1(static_cast<int16_t>(finalOutput));

        // ----------------------------------------------------
        // Lecture des commandes de transport MIDI
        // ----------------------------------------------------

        if (midiResetRequested.exchange(false))
        {
            bitPosition = 0;
            midiSubTicks = 0;
            internalCounter = 0;
            lastMidiTicksSeen = midiClockTicks.load();
        }
        // ----------------------------------------------------
        // Lecture des commandes physiques
        // ----------------------------------------------------

        const Switch mode = SwitchVal();
const int32_t physicalMainValue = KnobVal(Main);
const int32_t physicalXValue = KnobVal(X);
const int32_t yValue = KnobVal(Y);

// Gestion relative des effets en mode Down.
if (mode == Down && previousMode != Down)
{
    previousEffectsMainPhysical = physicalMainValue;
    previousEffectsXPhysical = physicalXValue;
}

if (mode == Down)
{
    const int32_t deltaMain =
        physicalMainValue - previousEffectsMainPhysical;

    const int32_t deltaX =
        physicalXValue - previousEffectsXPhysical;

    effectsDryWet += static_cast<float>(deltaMain) / 4095.0f;
    rustCorrosionMix += static_cast<float>(deltaX) / 4095.0f;

    if (effectsDryWet < 0.0f) effectsDryWet = 0.0f;
    if (effectsDryWet > 1.0f) effectsDryWet = 1.0f;

    if (rustCorrosionMix < 0.0f) rustCorrosionMix = 0.0f;
    if (rustCorrosionMix > 1.0f) rustCorrosionMix = 1.0f;
}

previousEffectsMainPhysical = physicalMainValue;
previousEffectsXPhysical = physicalXValue;


// ----------------------------------------------------
// Séparation des commandes Séquenceur / Effets
// ----------------------------------------------------

if (!controlPickupInitialized)
{
    sequencerMainValue = physicalMainValue;
    sequencerXValue = physicalXValue;
    controlPickupInitialized = true;
}

// Passage de Play vers Effets.
if (previousMode == Middle && mode == Down)
{
    mainPickupTarget = sequencerMainValue;
    xPickupTarget = sequencerXValue;

    mainPickupActive = true;
    xPickupActive = true;
}

// Retour des Effets vers Play.
if (previousMode == Down && mode == Middle)
{
    mainPickupActive = true;
    xPickupActive = true;
}

// En mode Play, les potentiomètres doivent retrouver
// leurs anciennes positions avant de reprendre le contrôle.
if (mode == Middle)
{
    if (mainPickupActive)
    {
        if (abs(physicalMainValue - mainPickupTarget) <= 16)
            mainPickupActive = false;
    }
    else
    {
        sequencerMainValue = physicalMainValue;
    }

    if (xPickupActive)
    {
        if (abs(physicalXValue - xPickupTarget) <= 16)
            xPickupActive = false;
    }
    else
    {
        sequencerXValue = physicalXValue;
    }
}

// En mode Stop, on prépare une reprise sans saut.
if (mode == Up)
{
    sequencerMainValue = physicalMainValue;
    sequencerXValue = physicalXValue;
    mainPickupActive = false;
    xPickupActive = false;
}

        // Initialise la mémoire du Skip au démarrage.
// Initialisation des valeurs mémorisées.
if (!skipValueInitialized)
{
    currentBpm = 120;

    skipValue = yValue;
    previousMode = mode;

    tempoPickupTarget =
        static_cast<int32_t>(((currentBpm - 20) * 4095) / 280);

    // Si le module démarre avec Z en bas,
    // Y doit retrouver la position correspondant à 120 BPM.
    tempoPickupActive = (mode == Down);

    skipValueInitialized = true;
}

// Passage de Play à Tempo.
if (previousMode == Middle && mode == Down)
{
    tempoPickupTarget =
        static_cast<int32_t>(((currentBpm - 20) * 4095) / 280);

    tempoPickupActive = true;
}

// Passage de Tempo à Play.
if (previousMode == Down && mode == Middle)
{
    skipPickupTarget = skipValue;
    skipPickupActive = true;
}

// Gestion du Skip en mode Play.
if (mode == Middle)
{
    if (skipPickupActive)
    {
        if (abs(yValue - skipPickupTarget) <= 16)
        {
            skipValue = yValue;
            skipPickupActive = false;
        }
    }
    else
    {
        skipValue = yValue;
    }
}

// Gestion du tempo en mode paramètre.
if (mode == Down)
{
    if (tempoPickupActive)
    {
        if (abs(yValue - tempoPickupTarget) <= 16)
        {
            currentBpm = 20 + static_cast<int>((yValue * 280) / 4095);
            tempoPickupActive = false;
        }
    }
    else
    {
        currentBpm = 20 + static_cast<int>((yValue * 280) / 4095);
    }
}

// Mémorisation de l'état actuel.
previousMode = mode;

        // Middle : arrêt complet du séquenceur.
        if (mode == Up)
        {
            PulseOut1(false);
            PulseOut2(false);

            pulse1Remaining = 0;
            pulse2Remaining = 0;

            envelope = 0.0f;
            CVOut2Millivolts(0);

            return;
        }

        // ----------------------------------------------------
        // Détection de l'horloge MIDI
        // ----------------------------------------------------

        // On vérifie le délai toutes les 256 périodes audio.
        if ((sampleCounter & 255u) == 0)
        {
            const uint32_t now = time_us_32();
            const uint32_t last = lastMidiClockUs.load();

            midiActiveCached =
                midiClockEverSeen.load()
                && static_cast<uint32_t>(now - last) < MIDI_CLOCK_TIMEOUT_US
                && !midiTransportStopped.load();
        }

        // ----------------------------------------------------
        // Choix de la source d'horloge
        //
        // Priorité :
        // 1. MIDI Clock
        // 2. Pulse 1 IN
        // 3. Horloge interne
        // ----------------------------------------------------

        ClockSource source;

        if (midiActiveCached)
        {
            source = ClockSource::Midi;
        }
        else if (Connected(Pulse1))
        {
            source = ClockSource::External;
        }
        else
        {
            source = ClockSource::Internal;
        }

        if (source != currentClockSource)
        {
            internalLedCounter = 0;
            midiLedTicks = 0;
// Mémorise le dernier tempo MIDI pour l'horloge interne.
if (source != ClockSource::Midi
    && mode != Down)
{
    const int receivedBpm = midiBpmReceived.load();

    if (receivedBpm > 0)
    {
        currentBpm = receivedBpm;
    }
}
            currentClockSource = source;
            midiSubTicks = 0;
            internalCounter = 0;
            lastStepSample = 0;
            lastExternalPulseSample = 0;
            nextExternalStepSample = 0;
            externalSubstepPeriod = 0;
            externalPulseSeen = false;

            // Évite de rejouer des ticks MIDI accumulés
            // avant le changement de source.
            lastMidiTicksSeen = midiClockTicks.load();
        }

        // ----------------------------------------------------
        // Horloge interne
        // ----------------------------------------------------

       const bool doubleSpeed = PulseIn2();

uint32_t internalPeriod =
    (SAMPLE_RATE * 60u) /
    (static_cast<uint32_t>(currentBpm) * (doubleSpeed ? 8u : 4u));

if (internalPeriod < 1)
    internalPeriod = 1;

stepPeriodSamples = internalPeriod;

        bool stepEvent = false;

        // ----------------------------------------------------
        // Traitement de l'horloge MIDI
        // 24 ticks MIDI = 1 pas du séquenceur
        // ----------------------------------------------------

        if (source == ClockSource::Midi)
{
    const uint32_t ticksNow = midiClockTicks.load();
    const uint32_t delta = ticksNow - lastMidiTicksSeen;

    lastMidiTicksSeen = ticksNow;
    midiSubTicks += delta;

    // Flash LED5 toutes les 24 impulsions MIDI.
    midiLedTicks += delta;

    if (midiLedTicks >= 24)
    {
        midiLedTicks -= 24;
        LedOn(4);
        led5Remaining = 480;
    }

    // Avancement du séquenceur.
    const uint32_t ticksPerStep = doubleSpeed ? 3u : 6u;

    while (midiSubTicks >= ticksPerStep)
    {
        midiSubTicks -= ticksPerStep;
        stepEvent = true;
    }
}

        // ----------------------------------------------------
        // Traitement de l'horloge externe
        // Pulse 1 IN
        // ----------------------------------------------------

        else if (source == ClockSource::External)
{
    if (PulseIn1RisingEdge())
    {
        // Flash LED4 à chaque impulsion externe.
LedOn(3);
led4Remaining = 480;
        // Mesure l'intervalle entre deux impulsions externes.
        if (lastExternalPulseSample != 0)
        {
            const uint32_t externalPeriod =
                sampleCounter - lastExternalPulseSample;

            // Divise l'intervalle en quatre ou huit pas.
externalSubstepPeriod = externalPeriod / (doubleSpeed ? 8u : 4u);

            // Programme le prochain pas intermédiaire.
            nextExternalStepSample =
                sampleCounter + externalSubstepPeriod;
        }

        lastExternalPulseSample = sampleCounter;
        externalPulseSeen = true;

        // L'impulsion externe déclenche immédiatement un pas.
        stepEvent = true;
    }
    else if (externalPulseSeen &&
             externalSubstepPeriod > 0 &&
             sampleCounter >= nextExternalStepSample)
    {
        // Déclenche les pas intermédiaires.
        stepEvent = true;

        nextExternalStepSample += externalSubstepPeriod;
    }
}

        // ----------------------------------------------------
        // Traitement de l'horloge interne
        // ----------------------------------------------------

        else
{
    ++internalCounter;

    if (internalCounter >= internalPeriod)
    {
        internalCounter = 0;
        stepEvent = true;
    }

    // Flash LED3 une fois par temps (noire).
    ++internalLedCounter;

    const uint32_t internalLedPeriod =
        (SAMPLE_RATE * 60u) / static_cast<uint32_t>(currentBpm);

    if (internalLedCounter >= internalLedPeriod)
    {
        internalLedCounter = 0;
        LedOn(2);
        led3Remaining = 480;
    }
}

  //const bool doubleSpeed = PulseIn2();

        // ----------------------------------------------------
        // Déclenchement d'un pas
        // ----------------------------------------------------

        if (stepEvent)
{
    if (source != ClockSource::Internal)
    {
        if (lastStepSample != 0)
        {
            stepPeriodSamples = sampleCounter - lastStepSample;
        }

        lastStepSample = sampleCounter;
    }
    else
    {
        stepPeriodSamples = internalPeriod;
    }

    PlayStep(sequencerMainValue, skipValue);
}
        // ----------------------------------------------------
        // Gestion des impulsions de sortie
        // ----------------------------------------------------

        if (pulse1Remaining > 0)
        {
            --pulse1Remaining;

            if (pulse1Remaining == 0)
                PulseOut1(false);
        }

        if (pulse2Remaining > 0)
        {
            --pulse2Remaining;

            if (pulse2Remaining == 0)
                PulseOut2(false);
        }

        // ----------------------------------------------------
// Gestion des LEDs
// ----------------------------------------------------

if (led1Remaining > 0)
{
    --led1Remaining;

    if (led1Remaining == 0)
        LedOff(0);
}

if (led2Remaining > 0)
{
    --led2Remaining;

    if (led2Remaining == 0)
        LedOff(1);
}

if (led3Remaining > 0)
{
    --led3Remaining;

    if (led3Remaining == 0)
        LedOff(2);
}

if (led4Remaining > 0)
{
    --led4Remaining;

    if (led4Remaining == 0)
        LedOff(3);
}

if (led5Remaining > 0)
{
    --led5Remaining;

    if (led5Remaining == 0)
        LedOff(4);
}

        // ----------------------------------------------------
        // Enveloppe AD exponentielle
        // ----------------------------------------------------

        if (envelope > 0.0005f)
        {
            envelope *= envelopeCoefficient;

            if (envelope < 0.0005f)
                envelope = 0.0f;
        }
        else
        {
            envelope = 0.0f;
        }

        const int32_t envelopeMillivolts =
            static_cast<int32_t>(envelope * 5000.0f);

        CVOut2Millivolts(envelopeMillivolts);
    }

private:

    enum class ClockSource
    {
        None,
        Midi,
        External,
        Internal
    };

    // --------------------------------------------------------
    // Générateur pseudo-aléatoire
    // --------------------------------------------------------

    uint32_t randomState = 0xA341316C;

    uint32_t Random32()
    {
        uint32_t x = randomState;

        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;

        randomState = x;
        return x;
    }

    uint8_t Random8()
    {
        return static_cast<uint8_t>(Random32() & 0xFF);
    }

    // --------------------------------------------------------
// Générateur de bruit blanc
// --------------------------------------------------------

uint32_t noiseState = 0xC1234567;

// ----------------------------------------------------
// Paramètres des effets Rust / Corrosion
// ----------------------------------------------------

float effectsDryWet = 0.0f;
float rustCorrosionMix = 0.0f;
int32_t previousEffectsMainPhysical = 0;
int32_t previousEffectsXPhysical = 0;

int16_t GenerateWhiteNoise()
{
    uint32_t x = noiseState;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;

    noiseState = x;

    // Signal audio centré, amplitude modérée.
    return static_cast<int16_t>(
        static_cast<int32_t>(x & 0x03FFu) - 512
    );
}

/// ----------------------------------------------------
// Effet Rust : bitcrusher
// ----------------------------------------------------
float ProcessRust(float input, float amount)
{
    // Limitation du paramètre.
    if (amount < 0.0f) amount = 0.0f;
    if (amount > 1.0f) amount = 1.0f;

    // État du sample-and-hold et du générateur aléatoire.
    static uint32_t rustCounter = 0;
    static float rustSampleHold = 0.0f;
    static uint32_t rustRandomState = 0xA341316Cu;

    // Fréquence de rééchantillonnage :
    // de la fréquence audio normale à 9 kHz.
    const float targetRate =
        static_cast<float>(SAMPLE_RATE)
        - amount * (static_cast<float>(SAMPLE_RATE) - 9000.0f);

    uint32_t basePeriod =
        static_cast<uint32_t>(
            roundf(static_cast<float>(SAMPLE_RATE) / targetRate)
        );

    if (basePeriod < 1)
        basePeriod = 1;

    // Capture d'un nouvel échantillon lorsque le compteur arrive à zéro.
    if (rustCounter == 0)
    {
        rustSampleHold = input;

        // Générateur pseudo-aléatoire xorshift.
        rustRandomState ^= rustRandomState << 13;
        rustRandomState ^= rustRandomState >> 17;
        rustRandomState ^= rustRandomState << 5;

        // Jitter à 100 % : variation aléatoire de la période.
        const uint32_t jitterRange =
            static_cast<uint32_t>(
                roundf(static_cast<float>(basePeriod) * amount)
            );

        const uint32_t randomSpan = jitterRange * 2u + 1u;

        const int32_t jitter =
            static_cast<int32_t>(rustRandomState % randomSpan)
            - static_cast<int32_t>(jitterRange);

        int32_t nextPeriod =
            static_cast<int32_t>(basePeriod) + jitter;

        if (nextPeriod < 1)
            nextPeriod = 1;

        rustCounter = static_cast<uint32_t>(nextPeriod);
    }

    --rustCounter;

    // Quantification à résolution variable.
    float normalized = rustSampleHold / 2048.0f;

// Préamplification pour provoquer un hard clipping.
const float fuzzDrive = 1.0f + amount * 2.0f;

// Écrêtage brutal et symétrique.
    if (normalized > 1.0f)
    {
        normalized = 1.0f;
    }

    if (normalized < -1.0f)
    {
        normalized = -1.0f;
    }

    normalized *= fuzzDrive;

    const float bitDepth = 12.0f - amount * 11.0f;
    const float levels = powf(2.0f, bitDepth);
    const float maxIndex = levels - 1.0f;

    const float quantized =
        roundf((normalized + 1.0f) * 0.5f * maxIndex);

    const float crushed =
        (quantized / maxIndex) * 2.0f - 1.0f;

    return crushed * 2048.0f;
}
// ----------------------------------------------------
// Effet Corrosion : saturation
// ----------------------------------------------------

float ProcessCorrosion(float input, float amount)
{
    // Saturation progressive, de douce à très agressive.
    const float drive = 1.0f + amount * 30.0f;
    const float driven = input * drive;

    const float saturated =
        tanhf(driven / 2048.0f) * 2048.0f;

    return saturated;
}

    // --------------------------------------------------------
    // Registre circulaire de type Turing Machine
    // --------------------------------------------------------

    uint8_t turingSequence[32];
    uint32_t loopLength = 0;
    uint32_t bitPosition = 0;

    void InitialiseTuringRegister(int length)
{
    loopLength = length;
    bitPosition = 0;

    // Remplit la boucle avec des valeurs aléatoires sur 8 bits.
    for (uint32_t i = 0; i < loopLength; ++i)
    {
        turingSequence[i] =
            static_cast<uint8_t>(Random32() & 0xFF);
    }
}

uint8_t ReadTuringValue()
{
    const uint8_t value = turingSequence[bitPosition];

    bitPosition = (bitPosition + 1) % loopLength;

    return value;
}

    // --------------------------------------------------------
    // Déclenchement d'un pas
    // --------------------------------------------------------

    void PlayStep(int32_t mainValue, int32_t yValue)
    {
        uint8_t sequenceValue;

        // Main tout en bas : hasard libre, nouvelle valeur à chaque pas.
        if (mainValue < 128)
        {
            sequenceValue = Random8();
            loopLength = 0;
        }
        else
        {
            // Main règle la longueur de boucle de 2 à 32 pas.
            uint32_t requestedLength =
    2u + static_cast<uint32_t>(
        ((mainValue - 128) * 30) / (4095 - 128)
    );

if (requestedLength > 32u)
    requestedLength = 32u;

            // Une nouvelle longueur initialise un nouveau registre.
            if (requestedLength != loopLength)
                InitialiseTuringRegister(requestedLength);

            sequenceValue = ReadTuringValue();
        }

        // CV 1 suit toujours la valeur du séquenceur,
        // même si le pas est ensuite ignoré par le Skip.
        sequencerCV = (static_cast<int32_t>(sequenceValue) * 2047) / 255;

const int32_t cvMillivolts =
    (sequencerCV * 5000) / 2047;

CVOut1Millivolts(cvMillivolts);

        // En mode Down, tous les pas sont joués.
        // Y contrôle alors le tempo et non le Skip.
        const int skipPercent = (yValue * 95) / 4095;

        const int randomPercent =
            static_cast<int>(Random32() % 100);

        const bool skipped = randomPercent < skipPercent;

        if (skipped)
        {
            // Pas ignoré :
            // Pulse 2 uniquement, sans enveloppe.
            PulseOut2(true);
            pulse2Remaining = TRIGGER_LENGTH_SAMPLES;
                LedOn(1);
    led2Remaining = 480;
        }
        else
        {
            // Pas joué :
            // Pulse 1 et déclenchement de l'enveloppe.
            PulseOut1(true);
            pulse1Remaining = TRIGGER_LENGTH_SAMPLES;
            LedOn(0);
led1Remaining = 480;

            // L'enveloppe dure la moitié de la période du pas.
            uint32_t decaySamples = stepPeriodSamples / 2;

            if (decaySamples < 1)
                decaySamples = 1;

            envelope = 1.0f;
            envelopeCoefficient =
                expf(-5.0f / static_cast<float>(decaySamples));
        }
    }

    // --------------------------------------------------------
    // Variables de fonctionnement
    // --------------------------------------------------------

    uint32_t sampleCounter = 0;
int currentBpm = 120;
int32_t tempoPickupTarget = 0;
bool tempoPickupActive = false;

// Mémoire indépendante du réglage Skip.
int32_t skipValue = 0;
int32_t skipPickupTarget = 0;
bool skipPickupActive = false;
bool skipValueInitialized = false;
Switch previousMode = Up;

// Mémorisation des commandes du séquenceur pendant le mode effets.
int32_t sequencerMainValue = 0;
int32_t sequencerXValue = 0;

int32_t mainPickupTarget = 0;
int32_t xPickupTarget = 0;

bool mainPickupActive = false;
bool xPickupActive = false;
bool controlPickupInitialized = false;

    uint32_t internalCounter = 0;
    uint32_t internalLedCounter = 0;
    uint32_t stepPeriodSamples = SAMPLE_RATE / 2;
    uint32_t lastStepSample = 0;
    uint32_t lastExternalPulseSample = 0;
    uint32_t nextExternalStepSample = 0;
    uint32_t externalSubstepPeriod = 0;
    bool externalPulseSeen = false; 

    uint32_t lastMidiTicksSeen = 0;
    uint32_t midiSubTicks = 0;

    uint32_t pulse1Remaining = 0;
    uint32_t pulse2Remaining = 0;

    uint32_t led1Remaining = 0;
uint32_t led2Remaining = 0;
uint32_t led3Remaining = 0;
uint32_t led4Remaining = 0;
uint32_t led5Remaining = 0;
uint32_t midiLedTicks = 0;

    float envelope = 0.0f;
    float envelopeCoefficient = 1.0f;
    int32_t sequencerCV = 0;

    ClockSource currentClockSource = ClockSource::None;
    bool midiActiveCached = false;
};

// ============================================================
// INSTANCE GLOBALE
// ============================================================

TuringSequencer card;

// ============================================================
// CALLBACKS MIDI USB HOST
// ============================================================

extern "C"
{

void tuh_midi_mount_cb(
    uint8_t dev_addr,
    uint8_t in_ep,
    uint8_t out_ep,
    uint8_t num_cables_rx,
    uint16_t num_cables_tx)
{
    (void)dev_addr;
    (void)in_ep;
    (void)out_ep;
    (void)num_cables_rx;
    (void)num_cables_tx;
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    (void)dev_addr;
    (void)instance;

    midiClockEverSeen.store(false);
    midiTransportStopped.store(false);
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
    (void)num_packets;

    uint8_t packet[4];

    while (tuh_midi_packet_read(dev_addr, packet))
{
    // Les messages MIDI temps réel utilisent le CIN 0xF.
    const uint8_t cin = packet[0] & 0x0F;

    if (cin != 0x0F)
        continue;

    const uint8_t message = packet[1];

 if (message == 0xF8)
{
    // MIDI Clock : mesure de l'intervalle entre deux impulsions.
    const uint32_t now = time_us_32();
    const uint32_t previous = lastMidiClockUs.exchange(now);

    if (previous != 0)
    {
        const uint32_t interval = now - previous;

        // Filtre les intervalles aberrants.
        if (interval >= 1000 && interval <= 500000)
        {
            // Calcule et mémorise le dernier tempo MIDI.
            uint32_t midiBpm = 60000000u / (24u * interval);

            if (midiBpm < 20) midiBpm = 20;
            if (midiBpm > 300) midiBpm = 300;

            midiBpmReceived.store(static_cast<int>(midiBpm));
        }
    }

    midiClockTicks.fetch_add(1);
    midiClockEverSeen.store(true);
}
    else if (message == 0xFA)
    {
        // Start : remise à zéro et démarrage.
        midiTransportStopped.store(false);
        midiResetRequested.store(true);
    }
    else if (message == 0xFB)
    {
        // Continue : reprise sans remise à zéro.
        midiTransportStopped.store(false);
    }
    else if (message == 0xFC)
    {
        // Stop : arrêt du transport.
        midiTransportStopped.store(true);
    }
}
}

void tud_midi_rx_cb(uint8_t itf)
{
    (void)itf;

    uint8_t packet[4];

    while (tud_midi_packet_read(packet))
    {
        // Les messages MIDI temps réel utilisent le CIN 0xF.
        const uint8_t cin = packet[0] & 0x0F;

        if (cin != 0x0F)
            continue;

        const uint8_t message = packet[1];

        if (message == 0xF8)
{
    // MIDI Clock : mesure de l'intervalle entre deux impulsions.
    const uint32_t now = time_us_32();
    const uint32_t previous = lastMidiClockUs.exchange(now);

    if (previous != 0)
    {
        const uint32_t interval = now - previous;

        // Filtre les intervalles aberrants.
        if (interval >= 1000 && interval <= 500000)
        {
            // Calcule et mémorise le dernier tempo MIDI.
            uint32_t midiBpm = 60000000u / (24u * interval);

            if (midiBpm < 20) midiBpm = 20;
            if (midiBpm > 300) midiBpm = 300;

            midiBpmReceived.store(static_cast<int>(midiBpm));
        }
    }

    midiClockTicks.fetch_add(1);
    midiClockEverSeen.store(true);
}
        else if (message == 0xFA)
        {
            // Start : remise à zéro et démarrage.
            midiTransportStopped.store(false);
            midiResetRequested.store(true);
        }
        else if (message == 0xFB)
        {
            // Continue : reprise sans remise à zéro.
            midiTransportStopped.store(false);
        }
        else if (message == 0xFC)
        {
            // Stop : arrêt du transport.
            midiTransportStopped.store(true);
        }
    }
}

void tuh_midi_tx_cb(uint8_t dev_addr)
{
    (void)dev_addr;
}

} // extern "C"

// ============================================================
// CŒUR 1 : MOTEUR AUDIO
// ============================================================

void core1_entry()
{
    card.Run();
}

// ============================================================
// CŒUR 0 : USB MIDI ET INITIALISATION
// ============================================================

int main()
{
    // Même fréquence système que le firmware Clockwork.
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    sleep_ms(10);

    set_sys_clock_khz(192000, true);

    card.Initialise();

    // Lancement du moteur audio sur le cœur 1.
    multicore_launch_core1(core1_entry);

    // Détection du rôle USB de la carte.
    const ComputerCard::USBPowerState_t usbState =
        card.GetUSBPowerState();

    if (usbState == ComputerCard::DFP)
    {
        // La carte est hôte USB : elle reçoit l'horloge MIDI.
        tuh_init(0);

        while (true)
        {
            tuh_task();
            tight_loop_contents();
        }
    }
    else if (usbState == ComputerCard::UFP)
    {
        // La carte est périphérique USB.
        tud_init(0);

        while (true)
        {
            tud_task();
            tight_loop_contents();
        }
    }

    // État USB non pris en charge.
    while (true)
        tight_loop_contents();

    return 0;
}

#ifndef VISIBILITY_CONTROL_HPP_
#define VISIBILITY_CONTROL_HPP_

#ifndef MIDI_VISIBILITY_CONTROL_
#define MIDI_VISIBILITY_CONTROL_

#if defined _WIN32 || defined __CYGWIN_
#ifdef __GNUC_
#define MIDI_BOT_EXPORT _attribute__((dllexport))
#define MIDI_BOT_EXPORT _attribute__((dllimport))
#else
#define MIDI_BOT_EXPORT __declspec(dillexport)
#define MIDI_BOT_EXPORT __declspec(dllimport)
#endif
#ifndef MIDI_BOT_BUILDING_DLL
#define MIDI_BOT_PUBLIC MIDI_BOT_EXPORT
#else
#define MIDI_BOT_PUBLIC MIDI_BOT_IMPORT
#endif
#define MIDI_BOT_PUBLIC_TYPE MIDI_BOT_PUBLIC
#define MIDI_BOT_LOCAL
#else 
#define MIDI_BOT_EXPORT __attribute__((visibilty("default")))
#define MIDI_BOT_IMPORT
#if __GNUC_ >= 4
#define MIDI_BOT_PUBLIC __attribute__((visibility("default")))
#define MIDI_BOT_LOCAL __attribute_((visibility("hidden")))
#else
#define MIDI_BOT_PUBLIC
#define MIDI_BOT_LOCAL
#endif
#define MIDI_BOT_PUBLIC_TYPE
#endif


#endif //VISIBILITY_CONTROL_HPP_
















#endif //VISIBILITY_CONTROL_HPP_


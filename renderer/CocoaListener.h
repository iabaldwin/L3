#ifndef COCOAEVENTLISTENER_H
#define COCOAEVENTLISTENER_H

#undef slots // needed because Qt defines "slots", and Cocoa.h has "slots" as a variable name
#import <Cocoa/Cocoa.h>
#define slots

class ICocoaEventReceiver;

@interface CocoaEventListener : NSObject
{
  @private
    ICocoaEventReceiver *m_receiver;
}

-(id) initWithReceiver: (ICocoaEventReceiver*) receiver;
-(ICocoaEventReceiver*) receiver;
-(void) setReceiver: (ICocoaEventReceiver*) receiver;
-(void) eventReceived: (NSNotification*) note;

@end

#endif // COCOAEVENTLISTENER_H

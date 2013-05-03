#include "ExternalInterface.h"
#include <iostream>
#include <sstream>

namespace L3
{
namespace Visualisers
{
    bool ExternalInterface::onEvent( glv::Event::t e, glv::GLV& g)
    {
        if ( e==20 )
        {
            visibility ? this->disable(glv::Visible) : this->enable(glv::Visible);
            visibility = !visibility; 
  
            if (!visibility )
            {
                this->restore(); 
                this->focused(false);
                this->disable( glv::Focused );
                this->disable( glv::DrawBack );
                this->disable( glv::HitTest );
                return false;
            }
            else
            {
                this->maximize();
                this->focused(true);
                this->bringToFront();
                this->enable( glv::Focused );
                this->enable( glv::DrawBack );
                this->enable( glv::HitTest );
            }
        }

        const glv::Keyboard& k = g.keyboard();
        int key = k.key();
        float mx = g.mouse().xRel();

        // Intercept switch
        switch(e){
            case glv::Event::KeyDown:
            if( key == 96 ){
                return false;
			}
	
        }

        // Process  
        bool retval = true;
        //retval = retval && glv::TextView::onEvent(e,g);

        this->eventify( e, g );

        if ( e == glv::Event::KeyDown) 
        {
            const glv::Keyboard& k = g.keyboard();
            int key = k.key();

            if ( key == glv::Key::Return )
            {
                std::string current = mText.substr( 0, mPos );
                history.push_front(  "\n" + current );

                deleteText(0, mText.size());
                cursorPos(0);

                // Join it
                std::stringstream ss; 
                for ( std::list<std::string>::iterator it=history.begin(); it != history.end(); it++ )
                    ss << *it;

                mText = ss.str();
                
                std::cout << "<hist>" << std::endl;
                std::cout << ss.str() << std::endl;;
                std::cout << "</hist>" << std::endl;
  
                this->interface->execute( current );
            
            }
        }
        
        return retval;
    }

    bool ExternalInterface::eventify( glv::Event::t e, glv::GLV& g){

        const glv::Keyboard& k = g.keyboard();
        int key = k.key();
        float mx = g.mouse().xRel();

        switch(e){
            case glv::Event::KeyDown:
                if(k.ctrl()){
                    switch(key){
                        case 'a': selectAll(); return false;
                    }
                }
                else if(k.alt() || k.meta()){} // bubble if control key down

                else if(k.isPrint()){

                    // preserve current text if something is selected
                    std::string oldText;
                    int oldSel = mSel;
                    int oldPos = mPos;
                    if(mSel){
                        oldText = mText;
                        deleteSelected();
                    }

                    // No filter assigned or filter returns true
                    if(!mFilter || mFilter(getValue(), mPos, key)){
                        mText.insert(mPos, 1, k.key());
                        setValue(mText);
                        cursorPos(mPos+1);
                        return false;
                    }

                    // restore old text if the new character is invalid
                    if(!oldText.empty()){
                        mText = oldText;
                        mPos = oldPos;
                        mSel = oldSel;
                    }
                }
                else{
                    switch(key){
                        case glv::Key::Backspace:
                            if(textSelected()) deleteSelected();
                            else if(validPos()){
                                deleteText(mPos-1, 1);
                                cursorPos(mPos-1);
                            }
                            return false;

                        case glv::Key::Delete:
                            if(textSelected()) deleteSelected();
                            else if(mText.size()){
                                deleteText(mPos, 1);
                                cursorPos(mPos);
                            }
                            return false;

                        case glv::Key::Left:
                            if(k.shift()) select(mSel-1);
                            else cursorPos(mPos-1);
                            return false;

                        case glv::Key::Right:
                            if(k.shift()) select(mSel+1);
                            else cursorPos(mPos+1);
                            return false;

                        case glv::Key::Down: cursorPos(mText.size()); return false;
                        case glv::Key::Up:   cursorPos(0); return false;

                        case glv::Key::Enter:
                        case glv::Key::Return:
                                        notify(this, glv::Update::Action);
                                        return false;
                    }
                }
                break;

            case glv::Event::MouseDown:
                cursorPos(xToPos(mx));
            case glv::Event::MouseUp:
                return false;

            case glv::Event::MouseDrag:
                {
                    int p = xToPos(mx);
                    if(p >= mPos) select(p-mPos+1);
                    else select(p-mPos);
                    //printf("%d\n", mSel);
                }
                return false;

            default:;
        }

        return true;
    }


}
}

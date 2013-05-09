#include "GLVInterface.h"
#include <iostream>
#include <sstream>


namespace L3
{
namespace Visualisers
{
    bool GLVInterface::onEvent( glv::Event::t e, glv::GLV& g)
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

        // Process  text
        bool retval = true;
        retval = retval && this->handleText( e, g );

        if ( e == glv::Event::KeyDown) 
        {
            const glv::Keyboard& k = g.keyboard();
            int key = k.key();

            if ( key == glv::Key::Return )
            {
                // Get the current string
                std::string current = mText.substr( 3, mPos-3 );

                if ( current.size() == 0 )
                    return false;
                    
                history.push_front( current );
             
                // Interface: L3
                //this->L3_interface->execute( current );
                // Interface: Lua
                //bool interface_status = this->interface->execute( current );
                //if ( interface_status )
                    //history.push_front( this->interface->get_state() );

                for( std::list< L3::Interface* >::iterator it = interfaces.begin();
                        it != interfaces.end();
                        it++ )
                {
                    std::pair< bool, std::string > result = (*it)->execute( current );
                    
                    //if( !result.first )
                    //{
                        history.push_front( result.second );
                    //}

                }


                // Reset things
                deleteText(0, mText.size());

                int counter = 0;

                // Join it
                std::stringstream ss; 
                for ( std::list<std::string>::iterator it=history.begin(); it != history.end(); it++ )
                        ss <<  "[" << counter++ << "] " << *it << std::endl;

                mText = std::string(">> ") + std::string("\n") + ss.str();
                cursorPos(3);

            }
        }
        
        return retval;
    }

    bool GLVInterface::handleText( glv::Event::t e, glv::GLV& g){

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
                                if ( mPos >3 )
                                {
                                deleteText(mPos-1, 1);
                                cursorPos(mPos-1);
                            
                                }
                            }
                            return false;

                        case glv::Key::Delete:
                            if(textSelected()) deleteSelected();
                            else if(mText.size()){
                                if ( mPos > 3 )
                                {
                                deleteText(mPos, 1);
                                cursorPos(mPos);
                                }
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

                        //case glv::Key::Down: cursorPos(mText.size()); return false;
                        //case glv::Key::Up:   cursorPos(0); return false;

                        case glv::Key::Up:
                            if ( history.size() > 0 )
                            {
                                mText = ">> " + history.front();
                                cursorPos( mText.size() );
                                return false;
                            }
                        
                        case glv::Key::Enter:
                        case glv::Key::Return:
                                        notify(this, glv::Update::Action);
                                        return false;
                    }
                }
                break;

        
            default:;
        }

        return true;
    }


}
}

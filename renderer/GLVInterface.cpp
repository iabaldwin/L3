#include "GLVInterface.h"
#include <iostream>
#include <sstream>

int last_key = 0;
int current_history_index = 0;

namespace L3
{
namespace Visualisers
{

    GLVInterface::GLVInterface( glv::Rect rect ) : glv::TextView( rect )
    {
        visibility = false;

        // Full screen, but not visible
        this->maximize(); 
        this->disable(glv::Visible);

        // Always bubble events to top-level, so we can catch toggle
        this->enable( glv::AlwaysBubble );

        // Initialisze
        mText = ">> ";
        cursorPos(3);
   
        std::ifstream history( ".history" );
        if ( history.good() )
        {
            std::string line;
            while( getline( history, line ) )
                command_history.push_front( line );
        }

        history.close();
    }


    bool GLVInterface::onEvent( glv::Event::t e, glv::GLV& g)
    {
        // Dump history
        if ( e == glv::Event::Quit )
            dumpHistory(); 

        // Master disable
        if ( e== static_cast< glv::Event::t>( OVERLAY_TOGGLE ) )
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

                if ( current.size() > 0 )
                    full_history.push_front( current );
       
                for( std::list< L3::Interface* >::iterator it = interfaces.begin();
                        it != interfaces.end();
                        it++ )
                {
                    if( (*it)->match( current ) )
                    { 
                        std::pair< bool, std::string > result = (*it)->execute( current );
                   
                        // This is nasty
                        this->bringToFront();

                        if( result.first )  // Successful command
                            command_history.push_front( current );
                    
                        if( result.second.size() > 0 ) // String to be displayed
                            full_history.push_front( result.second );
                    }
                }


                // Reset things
                deleteText(0, mText.size());

                int counter = 0;

                // Join it
                std::stringstream ss; 
                for ( std::list<std::string>::iterator it=full_history.begin(); it != full_history.end(); it++ )
                        ss <<  "[" << counter++ << "] " << *it << std::endl;

                mText = std::string(">> ") + std::string("\n") + ss.str();
                cursorPos(3);

            }
        
            last_key = key;
        
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
                        //case 'a': selectAll(); return false;
                        case 'a': this->selectAll(); return false;
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
                        case glv::Key::Up:
                            
                            if ( command_history.size() > 0 )
                            {
                                std::string previous;

                                if ( last_key == glv::Key::Up )
                                {
                                    if( current_history_index != (command_history.size()-1) )
                                        current_history_index++;
                                }
                                else
                                    current_history_index=0;

                                previous = command_history[current_history_index];

                                int counter= 0;
                                std::stringstream ss; 
                                for ( std::list<std::string>::iterator it=full_history.begin(); it != full_history.end(); it++ )
                                    ss <<  "[" << counter++ << "] " << *it << std::endl;

                                mText = std::string(">> ")  + previous + std::string("\n") + ss.str();
                                cursorPos(3 + previous.size());

                                return false;
                            }

                        case glv::Key::Tab:
                            return false;

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

    void GLVInterface::selectAll()
    {
        cursorEnd();
        select(-mText.size()+3);
    }

    void GLVInterface::dumpHistory()
    {
        std::ofstream history( ".history" );

        std::copy( command_history.begin(), 
                    command_history.end(),
                    std::ostream_iterator< std::string >( history, "\n") );

        history.close();
    }
    
}
}

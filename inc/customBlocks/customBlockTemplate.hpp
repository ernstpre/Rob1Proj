#ifndef CUSTOMBLOCKTEMPLATE_HPP_
#define CUSTOMBLOCKTEMPLATE_HPP_

#include <eeros/control/Block.hpp>
// include header files for the inputs, outputs, and subblocks

using namespace eeros::control;

template <typename T = double>
class CustomBlockName : public Block
{
public:
    CustomBlockName() 
    {
        // Connect subblocks, initilize variables
    }

        // Implement getter functions for inputs and outputs

    virtual void run()
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks
        
    }

protected:
    // Define inputs, outputs, intermediate variables and subblocks
};

#endif //CUSTOMBLOCKTEMPLATE_HPP_

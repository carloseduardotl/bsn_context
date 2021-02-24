#ifndef FORMULA_HPP
#define FORMULA_HPP

#include <iostream>
#include <stdexcept>
#include <string>

#include "lepton/Lepton.h"

 
namespace bsn {
    namespace model {
        class Formula {
        
            public:
                Formula();
                Formula(const std::string& text);
                Formula(const std::string& text, const std::vector<std::string> terms, const std::vector<double> values);
                ~Formula();

                Formula(const Formula &);
                Formula &operator=(const Formula &);

                double evaluate();
                double apply(const std::vector<std::string> terms, const std::vector<double> values);
                Lepton::CompiledExpression getExpression() const;
                void setExpression(const Lepton::CompiledExpression &);

            private:
                Lepton::CompiledExpression expression;
                std::vector<std::string> terms;
                std::vector<double> values;

        };
    }
}

#endif
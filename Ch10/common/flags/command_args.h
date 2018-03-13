#ifndef G2O_COMMAND_ARGS_H
#define G2O_COMMAND_ARGS_H

#include <string>
#include <vector>
#include <iostream>
#include <sstream>

class CommandArgs
{
    public:
        struct CommandArgument
        {
            std::string name;
            std::string description;
            int type;
            void* data;
            bool parsed;
            bool optional;

            CommandArgument() : name(""), description(""), type(0), data(0), parsed(false), optional(false) {}
        };

    public:
        CommandArgs();
        virtual ~CommandArgs();
        
        bool parseArgs(int argc, char** argv, bool exitOnError = true);

        void param(const std::string& name, bool& p, bool defValue, const std::string& desc);

        void param(const std::string& name, int& p, int defValue, const std::string& desc);

        void param(const std::string& name, float& p, float defValue, const std::string& desc);

        void param(const std::string& name, double& p, double defValue, const std::string& desc);

        void param(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc);

        void param(const std::string& name, std::vector<int>& p, const std::vector<int>& defValue, const std::string& desc);

        void param(const std::string& name, std::vector<double>& p, const std::vector<double>& defValue, const std::string& desc);


        void paramLeftOver(const std::string& name, std::string& p, const std::string& defValue, const std::string& desc, bool optional = false);


        void printParams(std::ostream& os);

        const std::string& getBanner() const
        { return _banner; }
        void setBanner(const std::string& banner);

        void printHelp(std::ostream& os);

        bool parsedParam(const std::string& paramFlag) const;

    protected:
        std::vector<CommandArgument> _args;
        std::vector<CommandArgument> _leftOvers;
        std::vector<CommandArgument> _leftOversOptional;
        std::string _banner;
        std::string _progName;

        const char* type2str(int t) const;
        void str2arg(const std::string& input, CommandArgument& ca) const;
        std::string arg2str(const CommandArgument& ca) const;

        std::string trim(const std::string& s) const;
};

#endif

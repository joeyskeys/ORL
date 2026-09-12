#include "orl_exec.hpp"

#include "orl_graph_import.h"

#if defined(ORL_HAS_GRAPH_IO)
#include "graph_serialization.hpp"
#endif

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

struct Options {
    std::filesystem::path input;
    std::string entry = "compute";
    std::string module;
    std::optional<std::string> emit_ir;
    std::optional<std::string> emit_oro;
    std::vector<std::string> include_paths;
    std::vector<std::string> exported_functions;
    ORL::exec::Backend backend = ORL::exec::Backend::Cpu;
    bool entry_explicit = false;
    bool backend_explicit = false;
    bool print_ir = false;
    bool help = false;
};

void print_usage(std::ostream& output) {
    output
        << "Usage: orlc <input.orl> [options]\n"
        << "\n"
        << "Compile an external ORL source file.\n"
        << "\n"
        << "Options:\n"
        << "  -e, --entry <name>       Runtime entry function (default: compute)\n"
        << "  -I, --include <dir>     Add a directory for use/imported ORL modules\n"
        << "      --backend <name>    Compilation backend: cpu or cuda (default: cpu)\n"
        << "      --emit-ir <file>    Write generated LLVM/NVVM IR to a file\n"
        << "      --print-ir          Print generated IR to stdout\n"
        << "      --emit-oro <file>   Export analyzed functions as an .oro node library\n"
        << "      --module <name>     Module ID for --emit-oro\n"
        << "      --export <name>    Export only this function to .oro (repeatable)\n"
        << "  -h, --help              Show this help\n";
}

bool read_value(int argc, char** argv, int* index,
    std::string_view option, std::string* value, std::string* error)
{
    if (*index + 1 >= argc) {
        *error = std::string{option} + " requires a value";
        return false;
    }
    *value = argv[++*index];
    if (value->empty()) {
        *error = std::string{option} + " requires a non-empty value";
        return false;
    }
    return true;
}

bool parse_args(int argc, char** argv, Options* options, std::string* error) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "-h" || argument == "--help") {
            options->help = true;
            continue;
        }
        if (argument == "-e" || argument == "--entry") {
            if (!read_value(argc, argv, &index, argument, &options->entry, error)) {
                return false;
            }
            options->entry_explicit = true;
            continue;
        }
        if (argument == "-I" || argument == "--include") {
            std::string path;
            if (!read_value(argc, argv, &index, argument, &path, error)) {
                return false;
            }
            options->include_paths.push_back(std::move(path));
            continue;
        }
        if (argument.rfind("-I", 0) == 0 && argument.size() > 2) {
            options->include_paths.push_back(argument.substr(2));
            continue;
        }
        if (argument == "--backend") {
            std::string backend;
            if (!read_value(argc, argv, &index, argument, &backend, error)) {
                return false;
            }
            if (backend == "cpu") {
                options->backend = ORL::exec::Backend::Cpu;
            } else if (backend == "cuda") {
                options->backend = ORL::exec::Backend::Cuda;
            } else {
                *error = "Unknown backend '" + backend + "'; expected cpu or cuda";
                return false;
            }
            options->backend_explicit = true;
            continue;
        }
        if (argument == "--emit-ir") {
            std::string path;
            if (!read_value(argc, argv, &index, argument, &path, error)) {
                return false;
            }
            options->emit_ir = std::move(path);
            continue;
        }
        if (argument == "--print-ir") {
            options->print_ir = true;
            continue;
        }
        if (argument == "--emit-oro") {
            std::string path;
            if (!read_value(argc, argv, &index, argument, &path, error)) {
                return false;
            }
            options->emit_oro = std::move(path);
            continue;
        }
        if (argument == "--module") {
            if (!read_value(argc, argv, &index, argument, &options->module, error)) {
                return false;
            }
            continue;
        }
        if (argument == "--export") {
            std::string function;
            if (!read_value(argc, argv, &index, argument, &function, error)) {
                return false;
            }
            options->exported_functions.push_back(std::move(function));
            continue;
        }
        if (!argument.empty() && argument.front() == '-') {
            *error = "Unknown option: " + argument;
            return false;
        }
        if (!options->input.empty()) {
            *error = "Only one input ORL file may be specified";
            return false;
        }
        options->input = argument;
    }

    if (!options->help && options->input.empty()) {
        *error = "An input .orl file is required";
        return false;
    }
    return true;
}

bool read_source(const std::filesystem::path& path, std::string* source,
    std::string* error)
{
    std::ifstream input(path, std::ios::in | std::ios::binary);
    if (!input.is_open()) {
        *error = "Unable to open ORL source file: " + path.string();
        return false;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    if (!input && !input.eof()) {
        *error = "Failed to read ORL source file: " + path.string();
        return false;
    }
    *source = contents.str();
    return true;
}

void add_input_directory(Options* options) {
    const auto directory = options->input.parent_path();
    const std::string path = directory.empty() ? "." : directory.string();
    options->include_paths.insert(options->include_paths.begin(), path);
}

void print_runtime_errors(std::string_view stage,
    const std::vector<std::string>& errors)
{
    if (errors.empty()) {
        std::cerr << "orlc: " << stage << " failed\n";
        return;
    }
    for (const auto& error : errors) {
        std::cerr << "orlc: " << stage << ": " << error << '\n';
    }
}

void print_analysis_errors(std::string_view stage,
    const std::vector<orlcomp::AnalysisDiagnostic>& diagnostics)
{
    for (const auto& diagnostic : diagnostics) {
        std::cerr << "orlc: " << stage << ": " << diagnostic.code
                  << ": " << diagnostic.message;
        if (!diagnostic.source.file.empty()) {
            std::cerr << " (" << diagnostic.source.file;
            if (diagnostic.source.line != 0) {
                std::cerr << ":" << diagnostic.source.line;
            }
            std::cerr << ")";
        }
        std::cerr << '\n';
    }
}

bool compile_runtime(const Options& options, const std::string& source) {
    ORL::exec::CompileOptions compile_options;
    compile_options.entry_function = options.entry;
    compile_options.source_name = options.input.string();
    compile_options.include_paths = options.include_paths;

    const auto program = ORL::exec::OrlProgram::Compile(
        source, std::move(compile_options));
    if (!program.valid()) {
        print_runtime_errors("source compilation", program.errors());
        return false;
    }

    const auto execution = ORL::exec::OrlExecution::Create(
        program, options.backend);
    if (!execution.valid()) {
        print_runtime_errors("code generation", execution.errors());
        return false;
    }

    if (options.emit_ir.has_value()) {
        std::ofstream output(*options.emit_ir, std::ios::out | std::ios::binary);
        if (!output.is_open()) {
            std::cerr << "orlc: unable to open IR output file: "
                      << *options.emit_ir << '\n';
            return false;
        }
        output << execution.ir();
        if (!output) {
            std::cerr << "orlc: failed to write IR output file: "
                      << *options.emit_ir << '\n';
            return false;
        }
    }
    if (options.print_ir) {
        std::cout << execution.ir();
        if (!execution.ir().empty() && execution.ir().back() != '\n') {
            std::cout << '\n';
        }
    }
    if (options.emit_ir.has_value()) {
        std::cout << "orlc: wrote " << *options.emit_ir << '\n';
    }
    if (!options.print_ir && !options.emit_ir.has_value()) {
        std::cout << "orlc: compiled " << options.input.string()
                  << " entry '" << options.entry << "'\n";
    }
    return true;
}

bool export_oro(const Options& options, const std::string& source) {
#if !defined(ORL_HAS_GRAPH_IO)
    (void)options;
    (void)source;
    std::cerr << "orlc: --emit-oro is unavailable because graph I/O support "
                 "(RapidJSON) was not built\n";
    return false;
#else
    orlcomp::NodeImportOptions import_options;
    import_options.module_name = options.module;
    import_options.source_name = options.input.string();
    import_options.include_paths = options.include_paths;
    import_options.exported_functions = options.exported_functions;
    const auto imported = orlcomp::import_node_definitions(
        source, std::move(import_options));
    if (!imported.ok()) {
        print_analysis_errors("node import", imported.diagnostics);
        return false;
    }

    orlgraph::GraphModule module;
    module.module_id = options.module;
    std::vector<orlgraph::Diagnostic> diagnostics;
    if (!orlgraph::save_oro(*options.emit_oro, module,
            imported.registry, &diagnostics))
    {
        for (const auto& diagnostic : diagnostics) {
            std::cerr << "orlc: .oro export: " << diagnostic.code
                      << ": " << diagnostic.message << '\n';
        }
        return false;
    }
    std::cout << "orlc: wrote " << *options.emit_oro << " with "
              << imported.registry.definitions().size()
              << " node definition(s)\n";
    return true;
#endif
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    std::string error;
    if (!parse_args(argc, argv, &options, &error)) {
        std::cerr << "orlc: " << error << "\n\n";
        print_usage(std::cerr);
        return 2;
    }
    if (options.help) {
        print_usage(std::cout);
        return 0;
    }

    add_input_directory(&options);
    if (options.module.empty()) {
        options.module = options.input.stem().string();
        if (options.module.empty()) {
            options.module = "orl_module";
        }
    }

    std::string source;
    if (!read_source(options.input, &source, &error)) {
        std::cerr << "orlc: " << error << '\n';
        return 1;
    }

    const bool runtime_requested = !options.emit_oro.has_value()
        || options.entry_explicit
        || options.backend_explicit
        || options.emit_ir.has_value()
        || options.print_ir;
    if (runtime_requested && !compile_runtime(options, source)) {
        return 1;
    }
    if (options.emit_oro.has_value() && !export_oro(options, source)) {
        return 1;
    }
    return 0;
}

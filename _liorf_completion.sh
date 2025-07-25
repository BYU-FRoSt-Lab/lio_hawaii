#!/bin/bash

# Define possible launch types
_liorf_launch_types="ouster sbg6 sbg9 bentest"

# Define common options for the liorf function itself
_liorf_main_options="-t --tag -d --dont_play_bag -h --help"

# Define common ros2 bag play flags
# This list can be extended with more flags as needed
_liorf_bag_play_flags="\
--loop \
--rate \
--start-offset \
-p --start-paused \
--duration \
--max-qos-depth \
--max-liveliness-duration \
--playback-frequency \
--remap \
--storage-id \
--topics \
--qos-override-policy \
"

_liorf_comp() {
    local cur prev words cword
    # COMP_WORDS: An array of the words on the current command line.
    # COMP_CWORD: The index into ${COMP_WORDS} of the word containing the current cursor position.
    
    # Directly use built-in Bash completion variables
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    words=("${COMP_WORDS[@]}")
    cword="${COMP_CWORD}"

    local all_suggestions=()
    local section_allows_bag_flags=false # Becomes true if we are in the section where bag files/flags appear

    # Determine which section of arguments we are in
    # Iterate through all words *before* the current cursor position (cword)
    # to see if we've already passed the main options phase.
    for (( i=1; i < cword; i++ )); do
        local word_being_checked="${words[i]}"
        local next_word_being_checked="${words[i+1]}"

        # If it's the launch type, just continue (it's always first)
        if [[ "$i" -eq 1 ]]; then
            continue
        fi

        # If it's a main option that takes an argument (-t/--tag), skip its value
        if [[ "$word_being_checked" == "-t" || "$word_being_checked" == "--tag" ]]; then
            if [[ -n "$next_word_being_checked" ]] && [[ "$next_word_being_checked" != -* ]]; then
                ((i++)) # Skip the tag value
            fi
            continue
        fi
        
        # If it's a simple main option (-d, -h/--help), continue
        if [[ "$word_being_checked" == "-d" || "$word_being_checked" == "-h" || "$word_being_checked" == "--help" ]]; then
            continue
        fi

        # If we reach here and the word is hyphenated, it's a ros2 bag play flag.
        # This explicitly means we are now in the bag play flags section.
        if [[ "$word_being_checked" == -* ]]; then
            section_allows_bag_flags=true
            break # We've found a bag play flag, so we're definitively in that section
        fi

        # If we reach here and the word is NOT hyphenated (and not a main option),
        # it must be a bag file path. This means we've passed the main options section
        # and are now in the bag files/bag flags section.
        if [[ "$word_being_checked" != -* ]]; then
            section_allows_bag_flags=true
            # No break here, as there can be multiple bag files before flags.
        fi
    done

    # --- Suggestion Logic ---
    if [[ "$cword" -eq 1 ]]; then
        # First argument: launch types (ouster, sbg6, sbg9, bentest)
        all_suggestions=( $(compgen -W "${_liorf_launch_types}" -- "$cur") )
    elif [[ "$cur" == -* ]]; then
        # Current word starts with a hyphen: suggest flags based on which section we're in
        if [[ "$section_allows_bag_flags" == true ]]; then
            # After bag files/flags, suggest only ros2 bag play flags
            all_suggestions=( $(compgen -W "${_liorf_bag_play_flags}" -- "$cur") )
        else
            # Before bag files/flags, suggest only main options
            all_suggestions=( $(compgen -W "${_liorf_main_options}" -- "$cur") )
        fi
    else
        # Current word does NOT start with a hyphen (e.g., empty string, partial path, or a flag argument)
        
        # Special handling for argument to -t / --tag
        if [[ "$prev" == "-t" || "$prev" == "--tag" ]]; then
            # After -t/--tag, we expect a tag value. No file or general flag suggestions.
            COMPREPLY=()
            return 0
        fi
        
        # Otherwise, it's either a bag file path or an argument to a bag flag.
        # Primarily, suggest file/directory paths.
        _filedir # This populates COMPREPLY directly
        
        # Add specific arguments for flags if the previous word was such a flag.
        local specific_flag_args=()
        case "$prev" in
            --remap)
                specific_flag_args=( $(compgen -W "from:=to" -- "$cur") )
                ;;
            --storage-id)
                specific_flag_args=( $(compgen -W "sqlite3" -- "$cur") )
                ;;
            --qos-override-policy)
                specific_flag_args=( $(compgen -W "RosidlDurability RosidlHistory RosidlReliability" -- "$cur") )
                ;;
            # Add more cases for flags that have specific argument values
        esac

        # Append specific flag arguments to COMPREPLY.
        if [[ ${#specific_flag_args[@]} -gt 0 ]]; then
            COMPREPLY=( "${COMPREPLY[@]}" "${specific_flag_args[@]}" )
        fi
        
        return 0 # Exit the function as COMPREPLY is handled
    fi

    # If we reached here, it means we are in the initial argument or a hyphenated argument case
    # where we want to use 'all_suggestions'.
    COMPREPLY=( "${all_suggestions[@]}" )
}

# Register the completion function
complete -F _liorf_comp liorf

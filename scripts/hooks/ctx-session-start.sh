#!/usr/bin/env bash
# Injected into context at session start — reminds Claude to prime the context-mode
# knowledge base before doing any research or analysis.
cat <<'EOF'
<context-mode-reminder>
This is the rp2040_pio_stepper project. Before any research or file analysis, run ONE
ctx_batch_execute call to index session context. Suggested starter:

  commands:
    - label: "git log"        command: "git log --oneline -20"
    - label: "open issues"    command: "gh issue list --repo mrdunk/rp2040_pio_stepper"
    - label: "source tree"    command: "find src -name '*.c' -o -name '*.h' | grep -v build | sort"
  queries: [add any questions relevant to the current task]

Bash only for: git, mkdir, rm, mv, navigation.
Large output → ctx_execute. File analysis → ctx_execute_file. Follow-ups → ctx_search.
</context-mode-reminder>
EOF

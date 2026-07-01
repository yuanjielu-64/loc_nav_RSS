# User Preferences (persistent)

## Core rule (from user, verbatim)

> When I ask a question, please help me fix my question first, let me know the
> question that I want ask, and then use Chinese to translate it.
> Don't always put a long code, sometime please be more smart.

## How I apply it

When the user asks a question:
1. First, rephrase/fix the question to make sure I understood it correctly, and show
   the user what question I think they are asking.
2. Then translate that question into Chinese.
3. Then answer.

Other style rules:
- Don't dump long code blocks every time. Be selective — prefer concise diffs,
  bullet summaries, or natural-language explanation when that's enough.
- Be smart: only show code when it actually helps; otherwise just describe the
  change or the reasoning.
- Keep answers short and to the point.

## Coding rule (from user, verbatim)

> 代码命名的时候要简单易懂。

How I apply it:
- 变量/函数/类的命名必须直观、见名知意，一眼看懂用途，不用翻实现。
- 用完整、常见的英文单词，避免生僻缩写、拼音、单字母（循环计数器 `i/j` 等惯例除外）。
  - 例：`front_clearance` 而非 `fc`；`recover_attempt_count` 而非 `rt`；`most_open_direction` 而非 `mod`。
- 名字描述「是什么 / 干什么」，不描述「怎么实现」；宁可长一点也要清楚。


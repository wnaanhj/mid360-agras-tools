# 上传到GitHub

只上传包含本页的这个仓库目录：它的顶层有README.md、LICENSE、python、java和tools。父目录的本地隔离区、整理记录及原始工程都不要上传。

1. 在GitHub建立空仓库（例如mid360-agras-noncommercial-tools），许可证选项先留空，使用本目录的自定义LICENSE，不选择MIT/GPL模板覆盖它。
2. 先阅读README与发布范围，运行工具检查：`python -B tools/check_release.py`。检查的是受控文件、哈希及高风险材料，不构成法律审查或绝对无隐私保证。
3. 在本目录初始化Git，检查`git status --short`和`git diff --cached --stat`，确认没有build、运行日志、现场数据和二进制。
4. 再按GitHub给出的新仓库remote地址推送；本次整理没有创建仓库、添加remote或推送。
5. 仓库简介应写“Non-commercial source-available tools / 非商用源码公开工具”，不要使用“OSI open source”“完整可构建APK”等不符合当前范围的描述。

默认.gitignore只是一层保护，不会清除已跟踪文件。以后添加截图/地图/日志前应单独脱敏并核对是否拥有分发权。若补全GPL算法或引入第三方实现，必须重新检查组合许可，而不是只改README。

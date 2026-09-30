"""T8：音乐停止词护栏 `_music_stop_requested()`。

依据：`docs/plans/2026-09-18-wake-handoff-and-vision-design.md` §9.6 ——
「靠模型正确调用 `stop_music` 不可靠……加一层关键词护栏」，并且明确
**判据只放音乐词**，不能用光秃秃的「停下」（那句多半是在说小车）。

## 为什么只测这一个纯函数

`ConversationManager` 要起麦克风 / 喇叭 / ASR / TTS / ROS，根本没法单测
（见 `core/conversation.py` 的模块 docstring）。T8 把判据做成**模块级常量 + 纯函数**
（照项目里 `_ELICIT_NO` / `_asks_a_question` 的做法），就是为了让这段**口语判据**
能被这里钉住。接线的那半边（`_on_enter_thinking` 里命中就 `music.stop()`、
以及 TTS 打断那套）在本文件里**不测**。

## 本文件里最要紧的不是「命中」，是**不命中**

判据误伤的代价不是「音乐多放了一会儿」，而是「用户想让**车**停下，音乐被一起杀了」，
或者「用户只是想把音量关小，音乐没了」。所以「不命中」那几组才是红线，
`test_word_list_has_no_car_command_inside_it` 更是直接对着词表本身下断言。
"""

from __future__ import annotations

import pytest

from core.conversation import _MUSIC_STOP_WORDS, _music_stop_requested


# ===========================================================================
# 1. 应该命中
# ===========================================================================


@pytest.mark.parametrize(
    "text",
    [
        "别放了",
        "不听了",
        "关掉音乐",
        "停音乐",
    ],
)
def test_design_required_words_hit(text: str) -> None:
    """设计 §9.6 点名的四个词必须命中（这是护栏存在的理由）。"""
    assert _music_stop_requested(text) is True


@pytest.mark.parametrize(
    "text",
    [
        # ① 固定说法
        "不要放了",
        "不用放了",
        "别再放了",
        "不放了",
        "不想听了",
        # ② 动作在前 + 音乐名词
        "停一下音乐",
        "停止音乐",
        "停歌",
        "别放音乐",
        "别放歌",
        "不要放歌",
        "别唱了",
        "不要唱了",
        "关了音乐",
        "关上音乐",
        "关掉歌曲",
        "把歌关掉",
        # ③ 音乐名词在前
        "音乐关掉",
        "音乐停了",
        # ④ 「不听 <音乐名词>」
        "不听音乐",
        "不想听音乐",
        "不听歌",
    ],
)
def test_common_variants_hit(text: str) -> None:
    """词表里其它写法的抽查（覆盖范围是我定的，见 §9.6 允许「合理范围」）。"""
    assert _music_stop_requested(text) is True


@pytest.mark.parametrize(
    "text",
    [
        "我不想听音乐了",  # 口语里最常见的「不听了 + 对象」的完整说法
        "把音乐关掉",
        "帮我把歌关掉",
        "这首歌别放了",
        "音乐停一下",
    ],
)
def test_realistic_sentences_hit(text: str) -> None:
    """真实口语句子（词表是子串匹配，命中不必等于整句）。"""
    assert _music_stop_requested(text) is True


# ===========================================================================
# 2. ★ 绝不能命中 —— 这几条最重要
# ===========================================================================


@pytest.mark.parametrize(
    "text",
    [
        "停下",
        "停一下",
        "往前走",
    ],
)
def test_car_commands_must_not_hit(text: str) -> None:
    """★ 设计 §9.6 点名的三条：这是在说小车 / 在发新指令，绝不能把音乐一起杀掉。"""
    assert _music_stop_requested(text) is False


@pytest.mark.parametrize(
    "text",
    [
        "往前开",
        "往后退一点",
        "左转",
        "右转",
        "别动",
        "停车",
        "往前走两米",
        "先停下，我要下车",
    ],
)
def test_more_car_commands_must_not_hit(text: str) -> None:
    """小车指令的其它说法 —— 判据里只要沾上「停」「走」这类运动词，这里就该红。"""
    assert _music_stop_requested(text) is False


@pytest.mark.parametrize(
    "text",
    [
        "把音乐关小一点",  # ★ 「关小」是调音量，不是要停
        "音乐声音大一点",
        "这首歌叫什么",
        "我想听音乐",
        "放首歌",
        "再来一首",
        "换一首",
        "现在几点了",
        "今天天气怎么样",
        "你刚才说什么",
    ],
)
def test_other_sentences_must_not_hit(text: str) -> None:
    """与「不要听了」无关的句子：问句、新指令、调音量、闲聊。"""
    assert _music_stop_requested(text) is False


# ===========================================================================
# 3. 空值 / 脏输入不能炸
# ===========================================================================


@pytest.mark.parametrize("text", ["", None, "   ", "\t", "。。。", "，", "？"])
def test_empty_and_blank_are_false(text) -> None:
    """空串 / None / 全是空白或标点 → False，且**不抛异常**。"""
    assert _music_stop_requested(text) is False


# ===========================================================================
# 4. 标点 / 语气词 / 大小写
# ===========================================================================


@pytest.mark.parametrize(
    "text",
    [
        "别放了。",
        "别放了！",
        "嗯，别放了。",
        "别放了  ",
        "关掉音乐，谢谢",
        "别、放、了",  # 极端脏输入：标点插在词中间
    ],
)
def test_punctuation_and_spacing_still_hit(text: str) -> None:
    """ASR 加不加标点不确定，标点 / 空白不该把命中挡掉。"""
    assert _music_stop_requested(text) is True


@pytest.mark.parametrize(
    "text",
    [
        "别放啦",  # ★ 「啦」是「了 + 啊」的合音，ASR 常这么转
        "关掉音乐吧",
        "不听了啊",
        "停音乐呀",
        "别放了啦",
    ],
)
def test_colloquial_particles_still_hit(text: str) -> None:
    """句尾语气词。★ 归一化只把「啦」**映射**成「了」——
    若改成「把语气词一律删掉」，「别放啦」会退化成危险的「别放」而**漏判**。"""
    assert _music_stop_requested(text) is True


def test_ascii_case_is_normalized_but_english_is_out_of_scope() -> None:
    """大小写：中文词表没有大小写，`.lower()` 是防御性的。

    这里钉两件事：① ASCII 混排不会把中文命中搞坏；② 判据**只认中文** ——
    英文不在词表里（本项目 ASR 是中文的）。将来若要支持英文，这条会红，
    提醒改动者先补词表、再改这里。
    """
    assert _music_stop_requested("别放了OK") is True
    assert _music_stop_requested("STOP") is False
    assert _music_stop_requested("STOP THE MUSIC") is False


# ===========================================================================
# 5. 对着词表本身的断言（变异自检的哨兵）
# ===========================================================================

# 小车指令的口径来自 `tools/car.py` 里 car_stop / move / turn 的描述。
_CAR_COMMANDS = (
    "停",
    "停下",
    "停一下",
    "停一停",
    "停车",
    "往前走",
    "往前开",
    "前进",
    "后退",
    "左转",
    "右转",
    "别动",
)


def test_word_list_has_no_car_command_inside_it() -> None:
    """★ 红线（设计 §9.6）：判据**只放音乐词**，绝不能放光秃秃的「停下」。

    这条是**对着常量本身**下断言的：只要有人往 `_MUSIC_STOP_WORDS` 里塞一个
    会出现在小车指令里的词（比如「停」「停下」），它立刻变红 —— 比逐句测更彻底，
    因为它对**整个词表 × 整个小车指令集**都成立。
    """
    for car in _CAR_COMMANDS:
        inside = [word for word in _MUSIC_STOP_WORDS if word in car]
        assert not inside, f"词表里的 {inside} 会命中小车指令 {car!r}（设计 §9.6 的红线）"


def test_word_list_shape() -> None:
    """词表本身：元组、非空、无首尾空白、无重复（重复 = 有人改错了）。"""
    assert isinstance(_MUSIC_STOP_WORDS, tuple)
    assert _MUSIC_STOP_WORDS, "词表被清空了 —— 护栏就彻底失效了"
    for word in _MUSIC_STOP_WORDS:
        assert isinstance(word, str) and word
        assert word == word.strip(), f"{word!r} 带了首尾空白（归一化会去掉它们，永远匹配不上）"
    assert len(set(_MUSIC_STOP_WORDS)) == len(_MUSIC_STOP_WORDS), "词表里有重复项"


def test_every_word_names_music_or_is_a_self_contained_no() -> None:
    """★ 绊线：词表里每一项要么**点名了音乐**，要么是「别 / 不 … 了」这种
    **自己就说完一句**的否定说法（「别放了」「不听了」）。

    出现第三种形状的词（「停」「停下」「别动」「放小点」）就得先证明它不会误伤
    小车 —— 所以它是**故意**挡在这里的。
    （本来想用「每个词 ≥3 字」当代理指标，但「停歌」2 字却是安全的，形状才是
    真正的不变量。）
    """
    for word in _MUSIC_STOP_WORDS:
        names_music = any(noun in word for noun in ("音乐", "歌", "曲"))
        self_contained_no = word.startswith(("别", "不")) and word.endswith("了")
        assert names_music or self_contained_no, (
            f"词表里的 {word!r} 形状不对：既没点名音乐，也不是「别 / 不 … 了」。"
            f"加它之前先证明它不会在「停下」「往前走」这类小车指令上误伤（设计 §9.6）"
        )

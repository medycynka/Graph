#ifndef ASTAR_MAZE_HPP
#define ASTAR_MAZE_HPP

#pragma once

#include <limits>
#include <cstdint>
#include <utility>
#include <unordered_set>


template <typename T, std::size_t SIZE>
struct Stack{
    T data_[SIZE]{};
    std::size_t pos = 0;

    template <typename ... Arg>
    void constexpr emplace_back(Arg &&... arg){
        data_[pos] = T{std::forward<Arg>(arg)...};
        ++pos;
    }

    constexpr T pop_back(){
        --pos;
        return data_[pos];
    }

    [[nodiscard]] constexpr bool empty() const { return pos == 0; }
    [[nodiscard]] constexpr std::size_t size() const { return pos; }
};

template <typename T, std::size_t COLS, std::size_t ROWS>
struct MazeGrid{
    constexpr static auto cols(){ return COLS; };
    constexpr static auto rows(){ return ROWS; };

    constexpr T &operator()(const std::size_t col, const std::size_t row){ return data_[col + row * COLS]; }
    constexpr const T &operator()(const std::size_t col, const std::size_t row) const { return data_[col + row * COLS]; }

    T data_[COLS * ROWS];
};

constexpr auto seed(){
    std::uint64_t shifted = 0;

    for(const auto c : __TIME__ ){
        shifted <<= 8;
        shifted |= c;
    }

    return shifted;
}

struct PCG{
    struct pcg32_random_t{
        std::uint64_t state=0;
        std::uint64_t inc=seed();
    };

    pcg32_random_t rng;
    typedef std::uint32_t result_type;

    constexpr result_type operator()(){ return pcg32_random_r(); }
    static result_type constexpr min(){ return std::numeric_limits<result_type>::min(); }
    static result_type constexpr max(){ return std::numeric_limits<result_type>::min(); }

private:
    constexpr std::uint32_t pcg32_random_r(){
        std::uint64_t oldstate = rng.state;
        rng.state = oldstate * 6364136223846793005ULL + (rng.inc | 1);
        std::uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
        std::uint32_t rot = oldstate >> 59u;
        return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    }

};

template <typename T, typename G>
constexpr auto distribution(G &g, T min, T max){
    const auto range = max - min + 1;
    const auto bias_rem = std::numeric_limits<T>::max() % range;
    const auto unbias_max = std::numeric_limits<T>::max() - bias_rem - 1;

    auto r = g();
    for(;r > unbias_max; r = g());

    return (r % range) + min;
}

struct Cell{
    bool left_open = false;
    bool right_open = false;
    bool up_open = false;
    bool down_open = false;
    bool visited = false;
};

enum class WallType{
    Empty, Vertical, Horizontal,
    UpperLeft, UpperRight, LowerLeft, LowerRight,
    RightTee, LeftTee, UpTee, DownTee, FourWay,
    Up, Down, Left, Right,
    Visited, Used
};

struct Loc{
    std::size_t col = 0;
    std::size_t row = 0;

    constexpr Loc(const std::size_t col_, const std::size_t row_): col(col_), row(row_){}
    constexpr Loc() = default;
};

template<std::size_t num_cols, std::size_t num_rows>
constexpr MazeGrid<Cell, num_cols, num_rows> make_maze(){
    PCG pcg;
    MazeGrid<Cell, num_cols, num_rows> M;
    std::size_t c = 0;
    std::size_t r = 0;
    Stack<Loc, num_cols * num_rows> history;
    history.emplace_back(c,r);
    
    while (!history.empty()){
        M(c, r).visited = true;
        Stack<char, 4> check{};

        if (c > 0 && M(c-1, r).visited == false) {
            check.emplace_back('L');
        }
        if (r > 0 && M(c, r-1).visited == false) {
            check.emplace_back('U');
        }
        if (c < num_cols-1 && M(c+1, r).visited == false) {
            check.emplace_back('R');
        }
        if (r < num_rows-1 && M(c, r+1).visited == false) {
            check.emplace_back('D');
        }

        if (!check.empty()) {
            history.emplace_back(c,r);

            for (auto num_pops = distribution(pcg, std::size_t(0), check.size() - 1); num_pops > 0; --num_pops){
                check.pop_back();
            }

            switch (check.pop_back()) {
                case 'L':
                    M(c, r).left_open = true;
                    --c;
                    M(c, r).right_open = true;
                    break;

                case 'U':
                    M(c, r).up_open = true;
                    --r;
                    M(c, r).down_open = true;
                    break;

                case 'R':
                    M(c, r).right_open = true;
                    ++c;
                    M(c, r).left_open = true;
                    break;

                case 'D':
                    M(c, r).down_open = true;
                    ++r;
                    M(c, r).up_open = true;
                    break;
            }
        } else {
            const auto last = history.pop_back();
            c = last.col;
            r = last.row;
        }
    }

    M(0,0).left_open = true;
    M(num_cols-1, num_rows-1).right_open = true;

    return M;
}

template<std::size_t num_cols, std::size_t num_rows>
constexpr MazeGrid<WallType, num_cols*2+1, num_rows*2+1> render_maze(const MazeGrid<Cell, num_cols, num_rows> &maze_data){
    MazeGrid<WallType, num_cols*2+1, num_rows*2+1> result{};
    
    for (std::size_t col = 0; col < num_cols; ++col){
        for (std::size_t row = 0; row < num_rows; ++row){
            const auto render_col = col * 2 + 1;
            const auto render_row = row * 2 + 1;
            const auto &cell = maze_data(col, row);

            if (!cell.up_open) {
                result(render_col, render_row-1) = WallType::Horizontal;
            }
            if (!cell.left_open) {
                result(render_col-1, render_row) = WallType::Vertical;
            }
            if (!cell.right_open) {
                result(render_col+1, render_row) = WallType::Vertical;
            }
            if (!cell.down_open) {
                result(render_col, render_row+1) = WallType::Horizontal;
            }
        }
    }

    for (std::size_t col = 0; col < result.cols(); col += 2){
        for (std::size_t row = 0; row < result.rows(); row += 2){
            const auto up     = (row == 0) ? false : result(col, row-1) != WallType::Empty;
            const auto left   = (col == 0) ? false : result(col-1, row) != WallType::Empty;
            const auto right  = (col == num_cols * 2) ? false : result(col+1, row) != WallType::Empty;
            const auto down   = (row == num_rows * 2) ? false : result(col, row+1) != WallType::Empty;

            if (up && right && down && left) {
                result(col, row) = WallType::FourWay;
            }
            if (up && right && down && !left) {
                result(col, row) = WallType::RightTee;
            }
            if (up && right && !down && left) {
                result(col, row) = WallType::UpTee;
            }
            if (up && !right && down && left) {
                result(col, row) = WallType::LeftTee;
            }
            if (!up && right && down && left) {
                result(col, row) = WallType::DownTee;
            }
            if (up && right && !down && !left) {
                result(col, row) = WallType::LowerLeft;
            }
            if (up && !right && !down && left) {
                result(col, row) = WallType::LowerRight;
            }
            if (!up && !right && down && left) {
                result(col, row) = WallType::UpperRight;
            }
            if (!up && right && down && !left) {
                result(col, row) = WallType::UpperLeft;
            }
            if (!up && right && !down && left) {
                result(col, row) = WallType::Horizontal;
            }
            if (up && !right && down && !left) {
                result(col, row) = WallType::Vertical;
            }
            if (up && !right && !down && !left) {
                result(col, row) = WallType::Up;
            }
            if (!up && right && !down && !left) {
                result(col, row) = WallType::Right;
            }
            if (!up && !right && down && !left) {
                result(col, row) = WallType::Down;
            }
            if (!up && !right && !down && left) {
                result(col, row) = WallType::Left;
            }
        }
    }
    
    return result;
}

#endif //ASTAR_MAZE_HPP
